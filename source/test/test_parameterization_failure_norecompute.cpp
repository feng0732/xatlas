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

// Unrecoverable parameterization test.
//
// This translation unit is compiled with XA_RECOMPUTE_CHARTS=0 (see the
// Makefile), which disables the piecewise recompute that normally rescues an
// invalid chart. It verifies the second requirement: a chart whose
// parameterization cannot be recovered is still reported as
// ChartType::Invalid in the output, rather than being silently treated as a
// successful parameterization, while its faces and source vertex mapping stay
// consistent with the rest of the output.

#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <set>
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

struct DomeMesh
{
	std::vector<float> positions;
	std::vector<uint32_t> indices;

	DomeMesh(int latSegments, int lonSegments, float fraction)
	{
		const float capPhi = fraction * kPi;
		positions.push_back(0.0f);
		positions.push_back(0.0f);
		positions.push_back(1.0f);
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
		for (int j = 0; j < lonSegments; j++)
			indices.insert(indices.end(), { 0u, (uint32_t)(ringStart(1) + j), (uint32_t)(ringStart(1) + (j + 1) % lonSegments) });
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

void testUnrecoverableChartMarkedInvalid()
{
	logf("testUnrecoverableChartMarkedInvalid (XA_RECOMPUTE_CHARTS=%d)\n", XA_RECOMPUTE_CHARTS);
	TEST_ASSERT(XA_RECOMPUTE_CHARTS == 0); // This build must have recompute disabled.

	const DomeMesh input(8, 16, 1.25f); // Same failure dome as the recoverable test.
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

	const xatlas::Mesh &mesh = atlas->meshes[0];
	int invalidCount = 0;
	int piecewiseCount = 0;
	std::set<uint32_t> coveredFaces;
	for (uint32_t c = 0; c < mesh.chartCount; c++) {
		const xatlas::Chart &chart = mesh.chartArray[c];
		if (chart.type == xatlas::ChartType::Invalid)
			invalidCount++;
		if (chart.type == xatlas::ChartType::Piecewise)
			piecewiseCount++;
		for (uint32_t i = 0; i < chart.faceCount; i++)
			TEST_ASSERT(coveredFaces.insert(chart.faceArray[i]).second);
	}

	// The failed chart must be explicitly marked invalid, and with recompute
	// disabled no piecewise sub-charts may be produced.
	TEST_ASSERT(invalidCount > 0);
	TEST_ASSERT(piecewiseCount == 0);

	// The invalid chart still accounts for its faces: nothing is silently
	// dropped, and the union of all charts is the full input face set.
	TEST_ASSERT(coveredFaces.size() == input.faceCount());
	for (uint32_t face = 0; face < input.faceCount(); face++)
		TEST_ASSERT(coveredFaces.count(face) != 0);

	// Source vertex mapping is preserved even for the invalid chart.
	TEST_ASSERT(mesh.indexCount == (uint32_t)input.indices.size());
	for (uint32_t i = 0; i < mesh.indexCount; i++) {
		const xatlas::Vertex &vertex = mesh.vertexArray[mesh.indexArray[i]];
		TEST_ASSERT(vertex.xref == input.indices[i]);
	}

	// Vertices attributed to the invalid chart must reference source vertices
	// used by that chart's faces, and must carry finite UVs from the failed
	// parameterization (a decidable, inspectable failure state).
	for (uint32_t c = 0; c < mesh.chartCount; c++) {
		if (mesh.chartArray[c].type != xatlas::ChartType::Invalid)
			continue;
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
	xatlas::Destroy(atlas);
	logf("   passed: %d chart(s) remain ChartType::Invalid, none reported as success\n", invalidCount);
}

} // namespace

int main()
{
	testUnrecoverableChartMarkedInvalid();
	logf("All unrecoverable parameterization tests passed.\n");
	return 0;
}
