#include <iostream>
#include <lodepng.h>
#include <fstream>
#include <sstream>
#include "Vector3.hpp"
#include "Vector2.hpp"

// The goal for this lab is to draw a triangle mesh loaded from an OBJ file from scratch.
// This time, we'll draw the mesh as a solid object, rather than just a wireframe or collection
// of vertex points.
//
// *** Your Code Here! ***
// Task 1: As with last week, this is actually in the header files - this time we'll need both 2D and 3D vector classes!
//         Implement the dot and cross products in Vector2.hpp and Vector3.hpp so we can use them to draw our mesh.

void setPixel(std::vector<uint8_t>& image, int x, int y, int width, int height, uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255)
{
	int pixelIdx = x + y * width;
	image[pixelIdx * 4 + 0] = r;
	image[pixelIdx * 4 + 1] = g;
	image[pixelIdx * 4 + 2] = b;
	image[pixelIdx * 4 + 3] = a;
}

// Task 2: Implement this barycentric-coordinate-based triangle drawing function, based on the algorithm
// described in the slides. I've broken down the steps involved here in comments added to the function.
void drawTriangle(std::vector<uint8_t>& image, int width, int height,
	const Vector2& p0, const Vector2& p1, const Vector2& p2,
	uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255)
{
	// Find a bounding box around the triangle


	int minX = (int)std::floor(std::min({ p0.x(), p1.x(), p2.x() }));
	int maxX = (int)std::ceil(std::max({ p0.x(), p1.x(), p2.x() }));
	int minY = (int)std::floor(std::min({ p0.y(), p1.y(), p2.y() }));
	int maxY = (int)std::ceil(std::max({ p0.y(), p1.y(), p2.y() }));


	// Check your minX, minY, maxX and maxY values don't lie outside the image!

	minX = std::min(std::max(minX,0), width-1);
	minY = std::min(std::max(minY,0), height-1);
	maxX = std::min(std::max(maxX,0), width-1);
	maxY = std::min(std::max(maxY,0), height-1);

	// Find vectors going along two edges of the triangle

	Vector2 edge1 = p1 - p0;
	Vector2 edge2 = p2 - p0;

	// Find the area of the triangle using a cross product.

	float triangleArea = 0.0f;

	triangleArea = 0.5 * edge1.cross(edge2);

	//Cull Negative Faces

	if (triangleArea < 0) {
		return;
	}

	//Loop for each pixel
	for(int x = minX; x <= maxX; ++x) 
		for (int y = minY; y <= maxY; ++y) {
			// Find the barycentric coordinates of the triangle!

			Vector2 p(x, y); // This is the 2D location of the pixel we are drawing.

			// Find the area of each of the three sub-triangles, using a cross product
			// YOUR CODE HERE - set the value of these three area variables.
			float a0 = (p1-p).cross(p2-p);
			float a1 = (p2-p).cross(p0-p);
			float a2 = (p0-p).cross(p1-p);

			// Find the barycentrics b0, b1, and b2 by dividing by triangle area.
			// YOUR CODE HERE - do the division and find b0, b1, b2.
			float b0 = a0/triangleArea;
			float b1 = a1 / triangleArea;
			float b2 = a2 / triangleArea;

			// Check if the sum of b0, b1, b2 is bigger than 1 (or ideally a number just over 1 
			// to account for numerical error).
			// If it's bigger, skip to the next pixel as we are outside the triangle.
			// YOUR CODE HERE

			float sum = b0 + b1 + b2;

			if (b0 < 0 || b1 < 0 || b2 < 0) {
				continue;
			}

			// Now we're sure we're inside the triangle, and we can draw this pixel!
			setPixel(image, x, y, width, height, r, g, b, a);
		}
}

int main()
{

	std::string outputFilename = "output.png";

	const int width = 512, height = 512;
	const int nChannels = 4;

	// Setting up an image buffer
	// This std::vector has one 8-bit value for each pixel in each row and column of the image, and
	// for each of the 4 channels (red, green, blue and alpha).
	// Remember 8-bit unsigned values can range from 0 to 255.
	std::vector<uint8_t> imageBuffer(height*width*nChannels);

	// This line sets the memory block occupied by the image to all zeros.
	memset(&imageBuffer[0], 0, width * height * nChannels * sizeof(uint8_t));

	std::string bunnyFilename = "../models/stanford_bunny_simplified.obj";

	std::ifstream bunnyFile(bunnyFilename);

	std::vector<Vector3> vertices;
	std::vector<std::vector<unsigned int>> faces;
	std::string line;
	while (!bunnyFile.eof())
	{
		std::getline(bunnyFile, line);
		std::stringstream lineSS(line.c_str());
		char lineStart;
		lineSS >> lineStart;
		char ignoreChar;
		if (lineStart == 'v') {
			Vector3 v;
			for (int i = 0; i < 3; ++i) lineSS >> v[i];
			vertices.push_back(v);
		}

		if (lineStart == 'f') {
			std::vector<unsigned int> face;
			unsigned int idx, idxTex, idxNorm;
			while (lineSS >> idx >> ignoreChar >> idxTex >> ignoreChar >> idxNorm) {
				face.push_back(idx - 1);
			}
			if (face.size() > 0) faces.push_back(face);
		}
	}

	//drawTriangle(imageBuffer, width, height, Vector2(10, 10), Vector2(100, 10), Vector2(10, 100), 255, 0, 0, 255);

	int bunnyNum = 1;

	for (const auto& face : faces) {
		Vector2 p0(vertices[face[0]].x() * 250 + width / 2, -vertices[face[0]].y() * 250 + height / 2);
		Vector2 p1(vertices[face[1]].x() * 250 + width / 2, -vertices[face[1]].y() * 250 + height / 2);
		Vector2 p2(vertices[face[2]].x() * 250 + width / 2, -vertices[face[2]].y() * 250 + height / 2);

		// Task 3: Draw the bunny!
		// Now you've finished your triangle drawing function, you'll see a red bunny, drawn using the code below:

		int r, g, b;

		switch (bunnyNum) {
		case 0:
			r = 255;
			g = 0;
			b = 0;
		case 1:

			r = (rand() % 256);
			g = (rand() % 256);
			b = (rand() % 256);
		}

		drawTriangle(imageBuffer, width, height, p0, p1, p2, r, g, b, 255);



		// This is a bit boring. Try replacing this code to draw two different bunny types.

		// Bunny 1: Random Colour Bunny
		// Assign a random colour to each triangle in the bunny. You can do this by making use of 
		// the rand() function in C++.
		// Hint: Remember rand() returns an int, but we want our colour values to lie between 0 and 255.
		// How can we make sure our random r, g, b values stick to the right range?

		// Bunny 2: (Sort of) Diffuse Lighting Bunny
		// For the final task we'll do a bit of a preview of session 5 on diffuse lighting.
		// The idea here is that we colour each triangle according to how much it points towards the camera.
		// 
		// To do this, first find a vector pointing out at 90 degrees from the triangle, and normalise it
		// This special perpendicular vector is called the triangle's *normal*.
		//
		// Once you have your normal, take the dot product with (0,0,1). This will effectively measure how much
		// the normal points down the positive z-axis.
		// Use this value to set the brightness of the triangle (remember to scale it back to the [0,255] range).
	}


	// *** Encoding image data ***
	// PNG files are compressed to save storage space. 
	// The lodepng::encode function applies this compression to the image buffer and saves the result 
	// to the filename given.
	int errorCode;
	errorCode = lodepng::encode(outputFilename, imageBuffer, width, height);
	if (errorCode) { // check the error code, in case an error occurred.
		std::cout << "lodepng error encoding image: " << lodepng_error_text(errorCode) << std::endl;
		return errorCode;
	}

	return 0;
}
