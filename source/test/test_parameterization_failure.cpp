/*
MIT License

Copyright (c) 2018-2020 Jonathan Young

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// Tests for the parameterization failure path.
//
// When a chart's initial parameterization is invalid (flipped triangles,
// zero area triangles or a self-intersecting boundary) xatlas recomputes the
// chart with the piecewise parameterizer, which splits the failed chart into
// valid sub-charts. The output state of that failure/recompute flow has to be
// decidable from the public API alone, so these tests assert on the published
// xatlas::Mesh / xatlas::Chart / xatlas::Vertex data rather than internals.
//
// The exercised geometry is an open spherical dome that wraps past 225
// degrees. Grown with relaxed chart options (a high max cost and no normal
// deviation weight) it produces a single highly curved chart whose LSCM
// parameterization self-intersects, forcing the piecewise recompute path.
//
// A second translation unit, test_parameterization_failure_norecompute.cpp,
// is built with XA_RECOMPUTE_CHARTS=0 and verifies that a chart which cannot
// be recovered stays marked ChartType::Invalid instead of being reported as a
// successful parameterization.

#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <xatlas.h>

namespace {

int logf(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	char buffer[2048];
	vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	return printf("%s", buffer);
}

#define TEST_ASSERT(_condition) \
	do { \
		if (!(_condition)) { \
			logf("[FAILED] '%s' %s %d\n", #_condition, __FILE__, __LINE__); \
			assert(_condition); \
		} \
	} while (0)

const float kPi = 3.14159265358979323846f;
const float kUvEpsilon = 1.0e-6f;

struct DomeMesh
{
	std::vector<float> positions; // xyz triples
	std::vector<uint32_t> indices;

	// Open spherical dome: a north pole plus LAT latitude rings, the last one
	// at fraction * pi polar angle, wound consistently outward. It has no
	// bottom cap and every triangle has a strictly positive 3D area.
	DomeMesh(int latSegments, int lonSegments, float fraction)
	{
		const float capPhi = fraction * kPi;
		positions.push_back(0.0f);
		positions.push_back(0.0f);
		positions.push_back(1.0f); // North pole, vertex 0.
		for (int i = 1; i <= latSegments; i++) {
			const float phi = capPhi * (float)i / (float)latSegments;
			for (int j = 0; j < lonSegments; j++) {
				const float theta = 2.0f * kPi * (float)j / (float)lonSegments;
				positions.push_back(sinf(phi) * cosf(theta));
				positions.push_back(sinf(phi) * sinf(theta));
				positions.push_back(cosf(phi));
			}
		}
		const auto ringStart = [lonSegments](int ring) { return 1 + (ring - 1) * lonSegments; };
		// Pole fan.
		for (int j = 0; j < lonSegments; j++)
			indices.insert(indices.end(), { 0u, (uint32_t)(ringStart(1) + j), (uint32_t)(ringStart(1) + (j + 1) % lonSegments) });
		// Ring quads.
		for (int i = 1; i < latSegments; i++) {
			const int r0 = ringStart(i);
			const int r1 = ringStart(i + 1);
			for (int j = 0; j < lonSegments; j++) {
				const int j1 = (j + 1) % lonSegments;
				indices.insert(indices.end(), { (uint32_t)(r0 + j), (uint32_t)(r1 + j), (uint32_t)(r1 + j1) });
				indices.insert(indices.end(), { (uint32_t)(r0 + j), (uint32_t)(r1 + j1), (uint32_t)(r0 + j1) });
			}
		}
	}

	uint32_t faceCount() const { return (uint32_t)(indices.size() / 3); }
	uint32_t vertexCount() const { return (uint32_t)(positions.size() / 3); }
};

// Signed doubled parametric (UV) area of a face. Positive/negative encodes
// winding; the requirement is that every face within a chart has the same
// sign, and that no face collapses to zero area.
float faceParametricArea(const xatlas::Mesh &mesh, uint32_t face)
{
	const xatlas::Vertex &v0 = mesh.vertexArray[mesh.indexArray[face * 3 + 0]];
	const xatlas::Vertex &v1 = mesh.vertexArray[mesh.indexArray[face * 3 + 1]];
	const xatlas::Vertex &v2 = mesh.vertexArray[mesh.indexArray[face * 3 + 2]];
	return (v1.uv[0] - v0.uv[0]) * (v2.uv[1] - v0.uv[1]) - (v2.uv[0] - v0.uv[0]) * (v1.uv[1] - v0.uv[1]);
}

struct ChartFaces
{
	std::set<uint32_t> faces;
	int positiveArea = 0;
	int negativeArea = 0;
	int zeroArea = 0;
};

// Returns false if a chart boundary edge properly crosses another boundary
// edge of the same chart in UV space. Edges sharing an endpoint (the normal
// boundary vertex case) are not counted as intersections.
bool chartBoundaryIntersects(const DomeMesh &input, const xatlas::Mesh &mesh, const xatlas::Chart &chart)
{
	struct Edge {
		uint32_t a, b; // Source mesh vertex indices.
	};
	// Boundary edges are the half-edges used by exactly one face of the chart.
	std::vector<Edge> edges;
	{
		std::set<std::pair<uint32_t, uint32_t>> edgeSet;
		std::map<std::pair<uint32_t, uint32_t>, int> useCount;
		for (uint32_t i = 0; i < chart.faceCount; i++) {
			const uint32_t face = chart.faceArray[i];
			for (int k = 0; k < 3; k++) {
				const uint32_t va = input.indices[face * 3 + k];
				const uint32_t vb = input.indices[face * 3 + (k + 1) % 3];
				const std::pair<uint32_t, uint32_t> key = va < vb ? std::make_pair(va, vb) : std::make_pair(vb, va);
				useCount[key]++;
				edgeSet.insert(key);
			}
		}
		for (const auto &entry : useCount) {
			if (entry.second == 1)
				edges.push_back({ entry.first.first, entry.first.second });
		}
	}
	if (edges.size() < 2)
		return false;
	// Resolve the UV assigned to each source vertex by this chart.
	std::map<uint32_t, std::pair<float, float>> uv;
	for (uint32_t i = 0; i < chart.faceCount; i++) {
		const uint32_t face = chart.faceArray[i];
		for (int k = 0; k < 3; k++) {
			const xatlas::Vertex &v = mesh.vertexArray[mesh.indexArray[face * 3 + k]];
			uv[v.xref] = std::make_pair(v.uv[0], v.uv[1]);
		}
	}
	const auto orient = [](const std::pair<float, float> &p, const std::pair<float, float> &q, const std::pair<float, float> &r) {
		const float value = (q.first - p.first) * (r.second - p.second) - (q.second - p.second) * (r.first - p.first);
		return (value > kUvEpsilon) - (value < -kUvEpsilon);
	};
	for (size_t i = 0; i < edges.size(); i++) {
		for (size_t j = i + 1; j < edges.size(); j++) {
			// Shared endpoints are expected along a boundary loop.
			if (edges[i].a == edges[j].a || edges[i].a == edges[j].b || edges[i].b == edges[j].a || edges[i].b == edges[j].b)
				continue;
			const std::pair<float, float> &a = uv[edges[i].a];
			const std::pair<float, float> &b = uv[edges[i].b];
			const std::pair<float, float> &c = uv[edges[j].a];
			const std::pair<float, float> &d = uv[edges[j].b];
			const int o1 = orient(a, b, c);
			const int o2 = orient(a, b, d);
			const int o3 = orient(c, d, a);
			const int o4 = orient(c, d, b);
			if (o1 == 0 || o2 == 0 || o3 == 0 || o4 == 0)
				continue; // Collinear/touching: not a proper crossing.
			if (o1 != o2 && o3 != o4)
				return true;
		}
	}
	return false;
}

// Build the atlas for the failure dome and collect per-chart face/area state.
// The relaxed options keep the curved region in one chart whose LSCM result is
// invalid, which is what drives the piecewise recompute.
xatlas::Atlas *generateFailureDome(const DomeMesh &input, std::vector<ChartFaces> &perChart)
{
	xatlas::Atlas *atlas = xatlas::Create();
	xatlas::SetPrint(logf, false);
	xatlas::MeshDecl decl;
	decl.vertexCount = input.vertexCount();
	decl.vertexPositionData = input.positions.data();
	decl.vertexPositionStride = sizeof(float) * 3;
	decl.indexCount = (uint32_t)input.indices.size();
	decl.indexData = input.indices.data();
	decl.indexFormat = xatlas::IndexFormat::UInt32;
	TEST_ASSERT(xatlas::AddMesh(atlas, decl) == xatlas::AddMeshError::Success);
	xatlas::ChartOptions options;
	options.maxCost = 1000.0f;
	options.normalDeviationWeight = 0.0f;
	xatlas::Generate(atlas, options);
	TEST_ASSERT(atlas->meshCount == 1);
	const xatlas::Mesh &mesh = atlas->meshes[0];
	perChart.resize(mesh.chartCount);
	for (uint32_t c = 0; c < mesh.chartCount; c++) {
		const xatlas::Chart &chart = mesh.chartArray[c];
		ChartFaces &state = perChart[c];
		for (uint32_t i = 0; i < chart.faceCount; i++) {
			const uint32_t face = chart.faceArray[i];
			state.faces.insert(face);
			const float area = faceParametricArea(mesh, face);
			if (area > kUvEpsilon)
				state.positiveArea++;
			else if (area < -kUvEpsilon)
				state.negativeArea++;
			else
				state.zeroArea++;
		}
	}
	return atlas;
}

// Test 1: a recoverable chart is split by the piecewise parameterizer. The
// resulting sub-charts must be valid (uniform winding, no zero area UV
// triangles, no self-intersecting boundary), together partition the input
// faces, and keep the source vertex of every output vertex.
void testRecoverableChartSplit()
{
	logf("testRecoverableChartSplit\n");
	const DomeMesh input(8, 16, 1.25f); // 225 degree open dome.
	std::vector<ChartFaces> perChart;
	xatlas::Atlas *atlas = generateFailureDome(input, perChart);
	const xatlas::Mesh &mesh = atlas->meshes[0];

	int piecewiseCount = 0;
	int invalidCount = 0;
	std::set<uint32_t> coveredFaces;
	for (uint32_t c = 0; c < mesh.chartCount; c++) {
		const xatlas::Chart &chart = mesh.chartArray[c];
		TEST_ASSERT(chart.faceCount > 0);
		if (chart.type == xatlas::ChartType::Piecewise)
			piecewiseCount++;
		if (chart.type == xatlas::ChartType::Invalid)
			invalidCount++;
		const ChartFaces &state = perChart[c];
		// No flipped faces: a chart's faces must all share one winding.
		// (The whole chart is allowed to be uniformly flipped.)
		TEST_ASSERT(state.positiveArea == 0 || state.negativeArea == 0);
		// No zero area triangles in the parameterization.
		TEST_ASSERT(state.zeroArea == 0);
		// No self-intersecting boundary.
		TEST_ASSERT(!chartBoundaryIntersects(input, mesh, chart));
		for (uint32_t face : state.faces) {
			// The face set must be a partition: no face assigned twice.
			TEST_ASSERT(coveredFaces.insert(face).second);
		}
	}
	// The failure path must actually have been exercised.
	TEST_ASSERT(piecewiseCount > 0);
	// Recovery means nothing is left invalid.
	TEST_ASSERT(invalidCount == 0);
	// The sub-charts collectively cover every face of the original chart's
	// input mesh exactly once.
	TEST_ASSERT(coveredFaces.size() == input.faceCount());
	for (uint32_t face = 0; face < input.faceCount(); face++)
		TEST_ASSERT(coveredFaces.count(face) != 0);

	// Vertex source preservation: every output vertex references an input
	// vertex, and the index order of the source mesh is retained.
	TEST_ASSERT(mesh.indexCount == (uint32_t)input.indices.size());
	for (uint32_t i = 0; i < mesh.indexCount; i++) {
		const xatlas::Vertex &vertex = mesh.vertexArray[mesh.indexArray[i]];
		TEST_ASSERT(vertex.xref == input.indices[i]);
		TEST_ASSERT(vertex.xref < input.vertexCount());
	}
	// Every charted vertex belongs to the chart that owns its faces.
	for (uint32_t c = 0; c < mesh.chartCount; c++) {
		std::set<uint32_t> chartSourceVertices;
		for (uint32_t i = 0; i < mesh.chartArray[c].faceCount; i++) {
			const uint32_t face = mesh.chartArray[c].faceArray[i];
			for (int k = 0; k < 3; k++)
				chartSourceVertices.insert(input.indices[face * 3 + k]);
		}
		for (uint32_t i = 0; i < mesh.indexCount; i++) {
			const xatlas::Vertex &vertex = mesh.vertexArray[mesh.indexArray[i]];
			if (vertex.chartIndex == (int32_t)c)
				TEST_ASSERT(chartSourceVertices.count(vertex.xref) != 0);
		}
	}
	xatlas::Destroy(atlas);
	logf("   passed: %d piecewise sub-charts cover %u faces\n", piecewiseCount, input.faceCount());
}

// Test 3: failure, split and final output must agree on each face and on the
// UV source of every vertex. Concretely:
//   - every input face appears in exactly one output chart, at the same index;
//   - every output index slot maps back to the same source vertex (xref);
//   - each output vertex's UV is carried by the chart named in its chartIndex,
//     i.e. no vertex is attributed to a chart that does not contain its face.
void testFailureSplitOutputConsistency()
{
	logf("testFailureSplitOutputConsistency\n");
	const DomeMesh input(8, 16, 1.25f);
	std::vector<ChartFaces> perChart;
	xatlas::Atlas *atlas = generateFailureDome(input, perChart);
	const xatlas::Mesh &mesh = atlas->meshes[0];

	// One chart per output slot, and the face sets form an exact partition of
	// the input faces.
	std::set<uint32_t> partitionFaces;
	for (uint32_t c = 0; c < mesh.chartCount; c++) {
		const xatlas::Chart &chart = mesh.chartArray[c];
		for (uint32_t i = 0; i < chart.faceCount; i++)
			TEST_ASSERT(partitionFaces.insert(chart.faceArray[i]).second);
	}
	TEST_ASSERT(partitionFaces.size() == input.faceCount());

	// Index order / xref agreement across the whole output.
	TEST_ASSERT(mesh.indexCount == (uint32_t)input.indices.size());
	for (uint32_t i = 0; i < mesh.indexCount; i++) {
		const xatlas::Vertex &vertex = mesh.vertexArray[mesh.indexArray[i]];
		TEST_ASSERT(vertex.xref == input.indices[i]);
	}

	// UV source agreement: build, per chart, the set of source vertices used by
	// the chart's faces and confirm every vertex attributed to that chart has
	// its xref in that set and a finite UV.
	for (uint32_t c = 0; c < mesh.chartCount; c++) {
		std::set<uint32_t> sourceVertices;
		for (uint32_t i = 0; i < mesh.chartArray[c].faceCount; i++) {
			const uint32_t face = mesh.chartArray[c].faceArray[i];
			for (int k = 0; k < 3; k++)
				sourceVertices.insert(input.indices[face * 3 + k]);
		}
		for (uint32_t i = 0; i < mesh.indexCount; i++) {
			const xatlas::Vertex &vertex = mesh.vertexArray[mesh.indexArray[i]];
			if (vertex.chartIndex != (int32_t)c)
				continue;
			TEST_ASSERT(sourceVertices.count(vertex.xref) != 0);
			TEST_ASSERT(isfinite(vertex.uv[0]) && isfinite(vertex.uv[1]));
		}
	}

	// A second Generate run on the same atlas must reproduce the same
	// face/xref mapping: the state has to be stable, not run-dependent.
	// Snapshot the first output first: Generate destroys and rebuilds the
	// output meshes, so references into atlas->meshes[0] do not survive it.
	const uint32_t firstIndexCount = mesh.indexCount;
	const uint32_t firstChartCount = mesh.chartCount;
	xatlas::ChartOptions options;
	options.maxCost = 1000.0f;
	options.normalDeviationWeight = 0.0f;
	xatlas::Generate(atlas, options);
	const xatlas::Mesh &regenerated = atlas->meshes[0];
	TEST_ASSERT(regenerated.indexCount == firstIndexCount);
	TEST_ASSERT(regenerated.chartCount == firstChartCount);
	std::set<uint32_t> regeneratedFaces;
	for (uint32_t c = 0; c < regenerated.chartCount; c++)
		for (uint32_t i = 0; i < regenerated.chartArray[c].faceCount; i++)
			regeneratedFaces.insert(regenerated.chartArray[c].faceArray[i]);
	TEST_ASSERT(regeneratedFaces == partitionFaces);
	for (uint32_t i = 0; i < regenerated.indexCount; i++) {
		const xatlas::Vertex &vertex = regenerated.vertexArray[regenerated.indexArray[i]];
		TEST_ASSERT(vertex.xref == input.indices[i]);
	}
	xatlas::Destroy(atlas);
	logf("   passed: face partition, xref and UV source mapping are consistent\n");
}

} // namespace

int main()
{
	testRecoverableChartSplit();
	testFailureSplitOutputConsistency();
	logf("All parameterization failure tests passed.\n");
	return 0;
}
