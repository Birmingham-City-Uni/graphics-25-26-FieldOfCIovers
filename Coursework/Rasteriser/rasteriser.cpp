#include <iostream>
#include <lodepng.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "Image.hpp"
#include "LinAlg.hpp"
#include "Light.hpp"
#include "Mesh.hpp"
#include "Shading.hpp"

enum ShadingMode {
	PHONG,BLINN_PHONG
};

struct Triangle {
	std::array<Eigen::Vector3f, 3> screen; // Coordinates of the triangle in screen space.
	std::array<Eigen::Vector3f, 3> verts; // Vertices of the triangle in world space.
	std::array<Eigen::Vector3f, 3> norms; // Normals of the triangle corners in world space.
	std::array<Eigen::Vector2f, 3> texs; // Texture coordinates of the triangle corners.
};

struct RObject {
	Mesh mesh;
	std::vector<uint8_t> texture;
	unsigned int texW, texH;
	Eigen::Matrix4f transform;
};

RObject loadObject(const std::string& meshPath, const std::string& texturePath, Eigen::Vector3f position, float rotX = 0.f, float rotY = 0.f, float rotZ = 0.f) {
	RObject obj;
	obj.mesh = loadMeshFile(meshPath);
	unsigned error = lodepng::decode(obj.texture, obj.texW, obj.texH, texturePath);
	if (error) {
		std::cerr << "Texture load failed for " << texturePath
			<< ": " << lodepng_error_text(error) << std::endl;
		// Set safe fallback dimensions
		obj.texW = 1; obj.texH = 1;
		obj.texture = { 255, 0, 255, 255 }; // magenta fallback
	}
	obj.transform = translationMatrix(position) * rotateXMatrix(rotX) * rotateYMatrix(rotY) * rotateZMatrix(rotZ);
	return obj;
}



float radians(const float degrees) {

	if (degrees == 0.0f) {
		float radians = degrees;
		return radians;
	}
	else {
		float radians = degrees * (M_PI / 180);
		return radians;;
	}
}

Eigen::Matrix4f projectionMatrix(int height, int width, float horzFov = 70.f * M_PI / 180.f, float zFar = 1500.f, float zNear = 0.01f)
{
	// ========= Subtask 1: Make a Projection Matrix ========
	// *** YOUR CODE HERE ***

	// Make a projection matrix following the formulation in the lecture slides, and using the provided parameters.
	// First, work out vertical FoV based on the horizontal FoV:
	float vertFov = 0.f;

	vertFov = (horzFov * height) / width;

	// Now construct the matrix.
	Eigen::Matrix4f projection;

	projection <<
		1 / (tan(0.5 * horzFov)), 0, 0, 0,
		0, 1 / (tan(0.5f * vertFov)), 0, 0,
		0, 0, zFar / (zFar - zNear), (-zFar * zNear) / (zFar - zNear),
		0, 0, 1, 0;

	return projection;
	// *** END YOUR CODE ***
}

void findScreenBoundingBox(const Triangle& t, int width, int height, int& minX, int& minY, int& maxX, int& maxY)
{
	// Find a bounding box around the triangle
	minX = std::min(std::min(t.screen[0].x(), t.screen[1].x()), t.screen[2].x());
	minY = std::min(std::min(t.screen[0].y(), t.screen[1].y()), t.screen[2].y());
	maxX = std::max(std::max(t.screen[0].x(), t.screen[1].x()), t.screen[2].x());
	maxY = std::max(std::max(t.screen[0].y(), t.screen[1].y()), t.screen[2].y());

	// Constrain it to lie within the image.
	minX = std::min(std::max(minX, 0), width - 1);
	maxX = std::min(std::max(maxX, 0), width - 1);
	minY = std::min(std::max(minY, 0), height - 1);
	maxY = std::min(std::max(maxY, 0), height - 1);
}


void drawTriangle(std::vector<uint8_t>& image, int width, int height,
	std::vector<float>& zBuffer,
	const Triangle& t,
	const std::vector<std::unique_ptr<Light>>& lights,
	const std::vector<uint8_t>& albedoTexture, int texWidth, int texHeight,
	const Eigen::Vector3f &specularColor, float specularExponent,ShadingMode shadingMode,const Eigen::Vector3f&camWorldPos)
{
	int minX, minY, maxX, maxY;
	findScreenBoundingBox(t, width, height, minX, minY, maxX, maxY);

	Eigen::Vector2f edge1 = v2(t.screen[2] - t.screen[0]);
	Eigen::Vector2f edge2 = v2(t.screen[1] - t.screen[0]);
	float triangleArea = 0.5f * vec2Cross(edge2, edge1);
	if (triangleArea < 0) {
		// Triangle is backfacing
		// Exit and quit drawing!
		return;
	}

	for (int x = minX; x <= maxX; ++x)
		for (int y = minY; y <= maxY; ++y) {
			Eigen::Vector2f p(x, y);

			// Find sub-triangle areas
			float a0 = 0.5f * fabsf(vec2Cross(v2(t.screen[1]) - v2(t.screen[2]), p - v2(t.screen[2])));
			float a1 = 0.5f * fabsf(vec2Cross(v2(t.screen[0]) - v2(t.screen[2]), p - v2(t.screen[2])));
			float a2 = 0.5f * fabsf(vec2Cross(v2(t.screen[0]) - v2(t.screen[1]), p - v2(t.screen[1])));

			// find barycentrics
			float b0 = a0 / triangleArea;
			float b1 = a1 / triangleArea;
			float b2 = a2 / triangleArea;

			// If outside triangle, exit early
			float sum = b0 + b1 + b2;
			if (sum > 1.0001) {
				continue;
			}

			float depth0 = t.screen[0].z(), depth1 = t.screen[1].z(), depth2 = t.screen[2].z();

			float depthP = powf((b0 / depth0 + b1 / depth1 + b2 / depth2), -1);

			Eigen::Vector3f worldP = depthP * (t.verts[0] * b0 / depth0 + t.verts[1] * b1 / depth1 + t.verts[2] * b2 / depth2);

			Eigen::Vector3f normP = (t.norms[0] * b0 / depth0 + t.norms[1] * b1 / depth1 + t.norms[2] * b2 / depth2);
			normP.normalize();
			
			float depth = depthP * (depth0 * b0 / depth0 + depth1 * b1 / depth1 + depth2 * b2 / depth2);

			// Work out where to sample in the zBuffer. Remember the zBuffer has only one channel,
			// so your index should be based on the pixel's x and y locations, and the width of the 
			// z buffer only.
			

			// If your depth is bigger than the current depth, skip drawing this pixel.
			// Otherwise, replace the zBuffer value at depthIdx with this depth.

			
			// Add code to calculate the texture coordinates corresponding to P, texP.
			// Use barycentric interpolation!
			Eigen::Vector2f texP = depthP * (t.texs[0] * b0 / depth0 + t.texs[1] * b1 / depth1 + t.texs[2] * b2 / depth2);

			// Convert this coordinate to a point in texture space
			// To do so, multiply by the texWidth and texHeight to get to the correct range.
			// Don't forget to flip the y coordinates! 
			int texR = (1.0f - texP.y()) * texHeight;
			int texC = (texP.x() * texWidth);
			// Handle the case where texR or texC end up outside the image!
			// There are different ways you could do this - for example using 
			// the modulo (%) operator to wrap around, or clamping to the edges.
			// Write your own code below to do this - once you're done you should be sure 
			// that 0 <= texC < texWidth and 0 <= texR < texHeight.

			texR = texR % texHeight;
			if (texR < 0) {
				texR = -texR;
			}
			texC = texC % texWidth;
			if (texC < 0) {
				texC = -texC;
			}

			Color texColor = getPixel(albedoTexture, texC, texR, texWidth, texHeight);

			if (texColor.a < 1) {
				continue;
			}

			Eigen::Vector3f albedo(texColor.r / 255.f, texColor.g / 255.f, texColor.b / 255.0f);

			Eigen::Vector3f color = Eigen::Vector3f::Zero();

			Eigen::Vector3f viewDir = (camWorldPos - worldP).normalized();

			int depthIdx = y * width + x;

			if (depth > zBuffer[depthIdx]) {
				continue;
			}

			if (texColor.a < 1) {
				continue;
			}

			zBuffer[depthIdx] = depth;


			// Iterate over lights, and sum to find colour.
			for (auto& light : lights) {

				// Work out the contribution from this light source, and add it to the color variable.

				// Work out the intensity of this light source, at the point worldP.
				Eigen::Vector3f lightIntensity = light->getIntensityAt(worldP);

				// We only need to do the following if the light isn't an ambient light.
				if (light->getType() != Light::Type::AMBIENT) {
					Eigen::Vector3f incomingLightDir = light->getDirection(worldP);

					float specularTerm;
					if (shadingMode == ShadingMode::PHONG) {
						specularTerm = phongSpecularTerm(incomingLightDir, normP, viewDir, specularExponent);
					}
					else {
						specularTerm = blinnPhongSpecularTerm(incomingLightDir, normP, viewDir, specularExponent);
					}

					Eigen::Vector3f specularOut = specularColor * specularTerm;
					specularOut = coeffWiseMultiply(specularOut, lightIntensity);

					// Take the dot product of the normal with the light direction.
					float dotProd = normP.dot(-incomingLightDir);

					// We don't want negative light - if dot product less than 0, set it to 0.
					dotProd = std::max(dotProd, 0.0f);

					// Multiply the light intensity by the dot product.
					Eigen::Vector3f diffuseOut = lightIntensity * dotProd;
					diffuseOut = coeffWiseMultiply(diffuseOut, albedo);

					color += specularOut;
					//color += diffuseOut;
					//color = (incomingLightDir + Eigen::Vector3f::Ones()) / 2;
				}
				else {
					// Light is ambient - just multiply light intensity with albedo.
					color += coeffWiseMultiply(lightIntensity, albedo);
				}
			}

			Color c;
			// Gamma-correcting colours.
			c.r = std::min(powf(color.x(), 1 / 2.2f), 1.0f) * 255;
			c.g = std::min(powf(color.y(), 1 / 2.2f), 1.0f) * 255;
			c.b = std::min(powf(color.z(), 1 / 2.2f), 1.0f) * 255;

			c.a = 255;

			setPixel(image, x, y, width, height, c);
		}
}



void drawMesh(std::vector<unsigned char>& image,
	std::vector<float>& zBuffer,
	const Mesh& mesh,
	const std::vector<uint8_t>& albedoTexture, int texWidth, int texHeight,
	const Eigen::Matrix4f& modelToWorld,
	const Eigen::Matrix4f& worldToClip,
	const std::vector<std::unique_ptr<Light>>& lights,
	int width, int height,
	const Eigen::Vector3f& specularColor, float specularExponent, ShadingMode shadingMode, const Eigen::Vector3f& camWorldPos)
{
	for (int i = 0; i < mesh.vFaces.size(); ++i) {


		Eigen::Vector3f
			v0 = mesh.verts[mesh.vFaces[i][0]],
			v1 = mesh.verts[mesh.vFaces[i][1]],
			v2 = mesh.verts[mesh.vFaces[i][2]];
		Eigen::Vector3f
			n0 = mesh.norms[mesh.nFaces[i][0]],
			n1 = mesh.norms[mesh.nFaces[i][1]],
			n2 = mesh.norms[mesh.nFaces[i][2]];

		Triangle t;
		t.verts[0] = (modelToWorld * vec3ToVec4(v0)).block<3, 1>(0, 0);
		t.verts[1] = (modelToWorld * vec3ToVec4(v1)).block<3, 1>(0, 0);
		t.verts[2] = (modelToWorld * vec3ToVec4(v2)).block<3, 1>(0, 0);

		// ======= Subtask 2: The Transformation Chain ======
		//*** YOUR CODE HERE ***
		// We've worked out the vertices in *world* space above.
		// You need to do the rest of the transformation chain!
		// Work out the vClip vectors, which are the vectors in clip space
		// Multiply by worldToClip, and do the perspective divide by the w component.
		// Check that all 3 vertices are in the clip box (-1 to 1 in x, y and z) and if not,
		// skip drawing this triangle.
		// Hint: use the outsideClipBox function to do this.
		// Finally, work out the screen space coordinates based on the image height and width.

		// Work out the clip space coordinates, by multiplying by worldToClip and doing the 
		// perspective divide.

		Eigen::Vector4f vClip0 = Eigen::Vector4f::Zero();
		Eigen::Vector4f vClip1 = Eigen::Vector4f::Zero();
		Eigen::Vector4f vClip2 = Eigen::Vector4f::Zero();

		vClip0 = worldToClip * vec3ToVec4(t.verts[0]);
		vClip1 = worldToClip * vec3ToVec4(t.verts[1]);
		vClip2 = worldToClip * vec3ToVec4(t.verts[2]);

		// Check that all 3 vertices are in the clip box (-1 to 1 in x, y and z) and if not,
		// skip drawing this triangle.
		// Hint: I've made a function outsideClipBox in LinAlg.hpp to help with this!

		vClip0 /= vClip0.w();
		vClip1 /= vClip1.w();
		vClip2 /= vClip2.w();

		if (outsideClipBox(vClip0) || outsideClipBox(vClip1) || outsideClipBox(vClip2)) {
			continue;
		}

		t.screen[0] = Eigen::Vector3f(
			width * (vClip0.x() + 1.0f) / 2,
			height * (-vClip0.y() + 1.0f) / 2,
			vClip0.z()
		);

		t.screen[1] = Eigen::Vector3f(
			width * (vClip1.x() + 1.0f) / 2,
			height * (-vClip1.y() + 1.0f) / 2,
			vClip1.z()
		);

		t.screen[2] = Eigen::Vector3f(
			width * (vClip2.x() + 1.0f) / 2,
			height * (-vClip2.y() + 1.0f) / 2,
			vClip2.z()
		);

		// Work out the screen space coordinates based on the image height and width.
		// Set the z component of each screen coordinate to be the clip-space z (for example
		// t.screen[0].z() == vClip0.z());

		// *** END YOUR CODE ***

		// transform the normals (using the inverse transpose of the upper 3x3 block)
		t.norms[0] = (modelToWorld.block<3, 3>(0, 0).inverse().transpose() * n0).normalized();
		t.norms[1] = (modelToWorld.block<3, 3>(0, 0).inverse().transpose() * n1).normalized();
		t.norms[2] = (modelToWorld.block<3, 3>(0, 0).inverse().transpose() * n2).normalized();

		t.texs[0] = mesh.texs[mesh.tFaces[i][0]];
		t.texs[1] = mesh.texs[mesh.tFaces[i][1]];
		t.texs[2] = mesh.texs[mesh.tFaces[i][2]];

		drawTriangle(image, width, height, zBuffer, t, lights, albedoTexture, texWidth, texHeight,specularColor,specularExponent,shadingMode,camWorldPos);
	}
}


int main()
{
	std::string outputFilename = "output.png";

	const int width = 1920, height = 1080;
	const int nChannels = 4;

	// Set up an image buffer
	std::vector<uint8_t> imageBuffer(height * width * nChannels);
	std::vector<float> zBuffer(height * width);

	// This line sets the image to black initially.
	Color black{ 0,0,0,255 };
	for (int r = 0; r < height; ++r) {
		for (int c = 0; c < width; ++c) {
			setPixel(imageBuffer, c, r, width, height, black);
			zBuffer[r * width + c] = 1.0f;
		}
	}

    // **** Replace this bit with your lovely rasteriser code ****

	Eigen::Matrix4f projection = projectionMatrix(height, width);

	// This matrix rotates the camera, tilting it down, then translates it up to make it look down on the scene.
	// Once your code is working, try changing this to move the camera around!
	Eigen::Matrix4f cameraToWorld = translationMatrix(Eigen::Vector3f(0.f, -10.0f, 2.f)) * rotateXMatrix(radians(-10.f)) * rotateZMatrix(radians(180.f)) * rotateYMatrix(radians(15.f));

	// The main important task = set up the worldToCamera and worldToClip matrices here!
	// Set up worldToCamera, based on cameraToWorld above
	Eigen::Matrix4f worldToCamera = cameraToWorld.inverse();
	// Set up worldToClip, using the projection and worldToCamera matrices
	Eigen::Matrix4f worldToClip = projection * worldToCamera;

	//Blinn-Phong Additions

	Eigen::Vector3f specularColor = Eigen::Vector3f::Ones();
	float specularExponent = 100.f;
	ShadingMode mode = ShadingMode::BLINN_PHONG;
	Eigen::Vector3f camWorldPos = (cameraToWorld * Eigen::Vector4f(0, 0, 0, 1)).block<3, 1>(0, 0);

	std::vector<std::unique_ptr<Light>> lights;
	// I've already added an ambient light for you!
	lights.emplace_back(new AmbientLight(Eigen::Vector3f(0.1f, 0.1f, 0.1f)));

	//lights.emplace_back(new PointLight(Eigen::Vector3f(1.1f, 1.1f, 1.1f), Eigen::Vector3f(0.f, 1.0f, 0.f)));
	lights.emplace_back(new DirectionalLight(Eigen::Vector3f(0.4f, 0.4f, 0.4f), Eigen::Vector3f(1.f, 0.f, 0.0f)));
	//lights.emplace_back(new SpotLight(Eigen::Vector3f(10.0f, 0.0f, 0.0f), Eigen::Vector3f(0.f, 1.f, 0.0f), Eigen::Vector3f(0, -1, 0), M_PI/8));

	
	//Updated Render Helper Function
	std::vector<RObject> objects = {
		loadObject("../models/powerArmourBody.obj","../models/tnwPowerArmourB.png",{-1.0f,5.0f,25.f},radians(180),radians(180),radians(0)),
		loadObject("../models/powerArmourHelmet.obj","../models/tnw_ncr_powerarmor_helmet.png",{-1.0f,2.65f,39.f},radians(180),radians(180),radians(0)),
		loadObject("../models/powerArmourGloves.obj","../models/t45_paglove_ncr_d.png",{0.5,2.5f,39.5f},radians(180),radians(-185),radians(0)),
		loadObject("../models/minigun.obj","../models/minigun.png",{8.75f,-4.f,7.25f},radians(-25),radians(150),radians(185)),
		loadObject("../models/ncrFlagMesh2.obj", "../models/nv_ncr_flag.png",{-14.5f,12.f,65.f},radians(180),radians(25),radians(-10)),
		loadObject("../models/ncrFlagPole.obj","../models/nv_legionflag.png",{-20.f,10.0f,32.5f},radians(180),radians(180),radians(0)),
		loadObject("../models/groundMesh.obj","../models/wastelanddirt.png",{0,25.f,500.f},radians(180),radians(0),radians(0)),
		loadObject("../models/sandbagMesh.obj","../models/sandbag.png",{-30.f,5.f,45.f},radians(180),radians(90),radians(0)),
		loadObject("../models/sandbagMesh.obj","../models/sandbag.png",{-0.f,7.5f,110.f},radians(180),radians(0),radians(0)),
		loadObject("../models/roadMesh.obj","../models/roadwasteland01.png",{-135.f,2.0f,100.f},radians(175),radians(135),radians(0)),
		loadObject("../models/ranchHouseM.obj","../models/ranchShack.png",{-700.f,30.0f,85.f},radians(175),radians(135),radians(0)),
		loadObject("../models/ranchHouseM.obj","../models/ranchShack.png",{-625.f,10.0f,65.f},radians(175),radians(175),radians(0)),
		loadObject("../models/goodspringHouseM.obj","../models/goodShack.png",{350.f,24.0f,50.f},radians(174),radians(180),radians(0)),
		loadObject("../models/watertowerMesh.obj","../models/watertower01.png",{375.f,24.0f,60.f},radians(174),radians(180),radians(0)),
		loadObject("../models/skyMesh.obj","../models/skyTex.png",{0.f,100.0f,0.f},radians(180),radians(180),radians(0)),
		loadObject("../models/treeMesh.obj","../models/Vurt_Jbark02.png",{47.5f,5.f,35.f},radians(180),radians(180),radians(0)),
		loadObject("../models/treeLeavesMesh.obj","../models/Vurt_JTreeTop42x.png",{47.5f,5.f,35.f},radians(180),radians(180),radians(0))
	};

	for (const auto& obj : objects) {
		drawMesh(imageBuffer, zBuffer, obj.mesh, obj.texture, obj.texW, obj.texH, obj.transform, worldToClip, lights, width, height, specularColor, specularExponent, mode, camWorldPos);
	}

	// For debug - draw point lights as colored circles so we can see where they are
	drawPointLights(imageBuffer, width, height, lights);

    // Save the image
    int errorCode;
    errorCode = lodepng::encode(outputFilename, imageBuffer, width, height);
    if (errorCode) { // check the error code, in case an error occurred.
		std::cout << "lodepng error encoding image: " << lodepng_error_text(errorCode) << std::endl;
		return errorCode;
    }

	saveZBufferImage("zBuffer.png", zBuffer, width, height);

    return 0;
}
