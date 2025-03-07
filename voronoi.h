#pragma once
#include "delaunator.hpp"
#include "chioriMath.h"

namespace chiori
{
	struct cVEdge
	{
		cVec2 origin;			// origin of the edge
		cVec2 endDir;			// if infinite, endDir is the direction of the edge, otherwise it is the end point
		bool infinite{ false }; // if true, the edge is infinite
		cVEdge(const cVec2& inOrigin = cVec2::zero, const cVec2& inEndDir = cVec2::zero, bool inInfinite = false) :
			origin{ inOrigin }, endDir{ inEndDir }, infinite{ inInfinite } {}

		bool operator==(const cVEdge& other) const { return origin == other.origin && endDir == other.endDir; }
	};

	struct cVVert
	{
		cVec2 site;								// the centriod/site of the vertex
		std::vector<unsigned> edgeIndices{};	// the indices of the edges that make up this vertex
		std::vector<unsigned> seedIndices{};	// the indices of the seed points that define this vertex (max 3)

		bool operator==(const cVVert& other) const
		{
			return (site == other.site) && (edgeIndices.size() == other.edgeIndices.size());
		}
	};

	struct cVCell
	{
		unsigned seedIndex{ 0 };			// the index of the cell seed
		std::vector<unsigned> vertices{};	// the indices of the vertices of the cell, in CCW order
		bool infinite{ false };				// does this cell have infinite area? (has 2 infinite edges that comprise it)
		unsigned infVertA, infVertB;		// the indices of the vertices that comprise the infinite edges of the cell (based on this cells vertices array!)
		unsigned infEdgeA, infEdgeB;		// the indices of the infinite edges that comprise the infinite cell walls segments
	};


	class cVoronoiDiagram
	{
	public:
		std::vector<cVVert> vertices{}; // Each vertex corresponds to a Voronoi region
		std::vector<cVEdge> edges{}; // All unique edges in the Voronoi diagram
		std::vector<cVec2> v_points{};	// The original points used to triangulate
		std::vector<std::vector<cVec2>> triangles{}; // the underlying triangulation of the v_points
			
		void create(const cVec2* points, unsigned count)
		{
			clear();
			v_points.resize(count);
			v_points.assign(points, points + count);
			triangles = triangulateDelaunator(v_points);

			std::vector<cVec2> circumcenters;
	
			for (size_t i = 0; i < triangles.size(); i++) {
				cVec2 circum = circumcenter(triangles[i][0], triangles[i][1], triangles[i][2]);
				circumcenters.push_back(circum);
			}
			for (size_t i = 0; i < triangles.size(); i++) {
				cVVert vertex;
				vertex.site = circumcenters[i];

				for (size_t j = 0; j < 3; j++) {

					size_t seedIndex = std::distance(v_points.begin(),
						std::find(v_points.begin(), v_points.end(), triangles[i][j]));
					vertex.seedIndices.push_back(seedIndex);  // Store the index of the seed point
					
					size_t edgeStart = j;
					size_t edgeEnd = (j + 1) % 3;

					size_t t0 = i;
					size_t t1 = findAdjacentTriangle(i, edgeStart, edgeEnd, triangles);

					cVEdge edge;
					if (t1 != std::numeric_limits<size_t>::max()) {
						edge = cVEdge(circumcenters[t0], circumcenters[t1]);
					}
					else {
						cVec2 midpoint = (triangles[i][edgeStart] + triangles[i][edgeEnd]) * 0.5f;
						cVec2 dir = -(triangles[i][edgeEnd] - triangles[i][edgeStart]).tangent();
						// outward pointing normal is used to create dir, while not 100% accurate, it
						// is very quick to compute and creates viable to use results
						// we also avoid normalization since this is an infinite edge
						// we only care about the direction of the vector
						edge = cVEdge(circumcenters[t0], dir, true);
					}

					auto itr = std::find_if(edges.begin(), edges.end(), [&](const cVEdge& e) {
						return e == edge;
						});
					int edgeIndex = -1;
					if (itr != edges.end()) {
						edgeIndex = std::distance(edges.begin(), itr);
					}
					else
					{
						edges.push_back(edge);
						edgeIndex = edges.size() - 1;
					}
					vertex.edgeIndices.push_back(edgeIndex);
				}

				vertices.push_back(vertex);
			}
		}

		void add(const cVec2& point, bool recreate = true)
		{
			auto itr = std::find_if(v_points.begin(), v_points.end(), [&](const cVec2& p) {
				return p == point;
				});
			if (itr != v_points.end())
				return; // duplicate point, ignore
			
			std::vector<cVec2> new_points = v_points;
			new_points.push_back(point);
			if (recreate)
				create(new_points.data(), new_points.size());
		}

		void remove(const cVec2& point, bool recreate = true)
		{
			auto itr = std::find_if(v_points.begin(), v_points.end(), [&](const cVec2& p) {
				return p == point;
				});
			if (itr == v_points.end())
				return; // point not in diagram, ignore

			v_points.erase(itr);
			std::vector<cVec2> new_points = v_points;
			if (recreate)
				create(new_points.data(), new_points.size());
		}
		
		void clear()
		{
			vertices.clear();
			edges.clear();
			v_points.clear();
			triangles.clear();
		}

		void transform(const cVec2& translation, const cRot& rotation, const cVec2& scale = cVec2::one);

		std::vector<cVCell> getCells() const;

		static std::vector<std::vector<cVec2>> triangulateDelaunator(const std::vector<cVec2>& points)
		{
			if (points.size() < 3)
				return {};

			// this removes duplicate points
			std::unordered_set<cVec2, cVec2Hash> pointSet{ points.begin(), points.end() };
			std::vector<float> coords;
			std::vector<std::vector<cVec2>> triangles;

			// Convert cVec2 points into flat double array
			for (const auto& p : pointSet) {
				coords.push_back(p.x);
				coords.push_back(p.y);
			}

			// Perform Delaunay triangulation
			delaunator::Delaunator d(coords);

			// Convert output triangles back into cVec2
			for (std::size_t i = 0; i < d.triangles.size(); i += 3) {
				cVec2 v0(d.coords[2 * d.triangles[i]], d.coords[2 * d.triangles[i] + 1]);
				cVec2 v1(d.coords[2 * d.triangles[i + 1]], d.coords[2 * d.triangles[i + 1] + 1]);
				cVec2 v2(d.coords[2 * d.triangles[i + 2]], d.coords[2 * d.triangles[i + 2] + 1]);

				triangles.push_back({ v0, v1, v2 });
			}

			return triangles;
		}
		static void save(const std::string& filename, const cVoronoiDiagram& diagram);
		static cVoronoiDiagram load(const std::string& filename);
	private:
		cVec2 circumcenter(cVec2 a, cVec2 b, cVec2 c)
		{
			float dA = a.dot(a);
			float dB = b.dot(b);
			float dC = c.dot(c);
			float det = 2.0f * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
			return { (dA * (b.y - c.y) + dB * (c.y - a.y) + dC * (a.y - b.y)) / det,
								 (dA * (c.x - b.x) + dB * (a.x - c.x) + dC * (b.x - a.x)) / det };
		}

		size_t findAdjacentTriangle(size_t current, size_t edgeStart, size_t edgeEnd, const std::vector<std::vector<cVec2>>& triangles)
		{
			for (size_t i = 0; i < triangles.size(); i++) {
				if (i == current) continue;
				for (size_t j = 0; j < 3; j++) {
					if ((triangles[i][j] == triangles[current][edgeEnd] && triangles[i][(j + 1) % 3] == triangles[current][edgeStart]) ||
						(triangles[i][(j + 1) % 3] == triangles[current][edgeEnd] && triangles[i][j] == triangles[current][edgeStart])) {
						return i;
					}
				}
			}
			return std::numeric_limits<size_t>::max();
		}
	};

	inline void cVoronoiDiagram::transform(const cVec2& translation, const cRot& rotation, const cVec2& scale)
	{
		cTransform xf{ translation, rotation };
		xf.scale = scale;
		for (cVVert& vert : vertices)
		{
			// scale, rotate, translate
			vert.site = cTransformVec(xf, vert.site);
		}

		for (cVEdge& edge : edges)
		{
			edge.origin = cTransformVec(xf, edge.origin);
			if (!edge.infinite)
				edge.endDir = cTransformVec(xf, edge.endDir);
		}

		for (cVec2& pt : v_points)
		{
			pt = cTransformVec(xf, pt);
		}
	}
	
	std::vector<cVec2> ClipVoronoiWithPolygon(
		const cVoronoiDiagram& inPattern, const cVec2* p_vertices, const cVec2* p_normals, int p_count);



	#define VORONOI_EXTENSION ".vdf"
	#define VORONOI_FOLDER_NAME "voronoi_data"
	#define CCCAST(x) reinterpret_cast<const char*>(x)
	#define CCAST(x) reinterpret_cast<char*>(x)
	namespace fs = std::filesystem;

	static fs::path getProjectPath()
	{
		fs::path path = fs::current_path();
		// **If "voronoi_data" was not found, create it in the current directory**
		if (path.filename() != VORONOI_FOLDER_NAME) {
			path = fs::current_path() / VORONOI_FOLDER_NAME;
			fs::create_directories(path); // Creates the folder if it doesn't exist
		}
		return path;
	}
	
	inline void cVoronoiDiagram::save(const std::string& filename, const cVoronoiDiagram& diagram)
	{
		fs::path filePath = getProjectPath() / (filename + VORONOI_EXTENSION);

		std::ofstream file(filePath, std::ios::binary);
		if (!file) {
			std::cerr << "Error: Could not open file for writing: " << filePath << "\n";
			return;
		}

		int numVerts = diagram.vertices.size();
		int numEdges = diagram.edges.size();
		int numPts = diagram.v_points.size();
		file.write(CCCAST(&numVerts), sizeof(int));
		file.write(CCCAST(&numEdges), sizeof(int));
		file.write(CCCAST(&numPts), sizeof(int));

		for (const auto& vert : diagram.vertices)
		{
			file.write(CCCAST(&vert.site), sizeof(cVec2));
			size_t edgeCount = vert.edgeIndices.size();
			file.write(CCCAST(&edgeCount), sizeof(size_t));
			file.write(CCCAST(vert.edgeIndices.data()), edgeCount * sizeof(size_t));
		}

		for (const auto& edge : diagram.edges)
		{
			file.write(CCCAST(&edge.origin), sizeof(cVec2));
			file.write(CCCAST(&edge.endDir), sizeof(cVec2));
			file.write(CCCAST(&edge.infinite), sizeof(bool));
		}

		for (const auto& pt : diagram.v_points)
		{
			file.write(CCCAST(&pt), sizeof(cVec2));
		}

		file.close();
	}

	inline cVoronoiDiagram cVoronoiDiagram::load(const std::string& filename)
	{
		fs::path filePath = getProjectPath() / (filename + VORONOI_EXTENSION);

		std::ifstream file(filePath, std::ios::binary);
		if (!file) {
			std::cerr << "Error: Could not open file for reading: " << filePath << "\n";
			return cVoronoiDiagram();
		}

		cVoronoiDiagram diagram;
		int numVerts, numEdges, numPts;
		file.read(CCAST(&numVerts), sizeof(int));
		file.read(CCAST(&numEdges), sizeof(int));
		file.read(CCAST(&numPts), sizeof(int));

		diagram.vertices.resize(numVerts);
		diagram.edges.resize(numEdges);
		diagram.v_points.resize(numPts);

		for (auto& vert : diagram.vertices)
		{
			file.read(CCAST(&vert.site), sizeof(cVec2));
			size_t edgeCount;
			file.read(CCAST(&edgeCount), sizeof(size_t));
			vert.edgeIndices.resize(edgeCount);
			file.read(CCAST(vert.edgeIndices.data()), edgeCount * sizeof(size_t));
		}

		for (auto& edge : diagram.edges)
		{
			file.read(CCAST(&edge.origin), sizeof(cVec2));
			file.read(CCAST(&edge.endDir), sizeof(cVec2));
			file.read(CCAST(&edge.infinite), sizeof(bool));
		}

		for (auto& pt : diagram.v_points)
		{
			file.read(CCAST(&pt), sizeof(cVec2));
		}

		file.close();
		return diagram;
	}
}