#include <Eigen/Dense>
#include <lodepng.h>
#include <json/json.hpp>
#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include "BVHNode.hpp"
#include "Triangle.hpp"
#include "Scene.hpp"
#include "Camera.hpp"
#include "PointLight.hpp"
#include "DirectionalLight.hpp"
#include "LambertianShader.hpp"
#include "TexturedLambertianShader.hpp"
#include "PhongShader.hpp"
#include "MirrorShader.hpp"
#include "TexCoordTestShader.hpp"
#include "Model.hpp"
#include <fstream>

/// <summary>
/// Load a JSON config file using the nlohmann library.
/// </summary>
nlohmann::json loadConfig(const std::string& filename)
{
	std::ifstream configStream(filename);
	nlohmann::json config = nlohmann::json::parse(configStream);
	return config;
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

struct ModelSpawner {
	Scene& scene;
	std::vector<std::shared_ptr<Model>> models;

	std::shared_ptr<BVHNode> addObject(const std::string& meshPath, const Shader* shader, int bvhDepth = 4, Eigen::Vector3f position = Eigen::Vector3f::Zero(), float rotX = 0.f, float rotY = 0.f, float rotZ = 0.f) {

		models.emplace_back(std::make_shared<Model>(meshPath.c_str()));
		std::shared_ptr <Model>& model = models.back();

		if (model->nfaces() == 0) {
			std::cerr << "WARNING: Model has no faces, skipping: " << meshPath << std::endl;
			models.pop_back();
			return nullptr;
		}

		Eigen::Matrix4f transform = makeTranslationMatrix(position) * rotateX(rotX) * rotateY(rotY) * rotateZ(rotZ);
		auto node = std::make_shared<BVHNode>(*model, shader, bvhDepth, transform);
		scene.renderables.push_back(node);
		return node;
	}
};

/// <summary>
/// Load an Eigen Vector3f from a config file.
/// Call as for example loadVec3FromConfig(config["myVector3"]);
/// </summary>
Eigen::Vector3f loadVec3FromConfig(const nlohmann::json& config)
{
	return Eigen::Vector3f(config[0], config[1], config[2]);
}

int main(int argc, char* argv[]) {

	// *** Load the config file ***
	auto config = loadConfig("../config/config.json");

	const int pixHeight = config["pixHeight"], pixWidth = config["pixWidth"];
	const int nChannels = 4;

	Eigen::Matrix4f camRot = rotateX(radians(-10.f)) * rotateY(radians(-15.f));
	Eigen::Vector3f cameraPos(0.f, -10.0f, 2.f);
	Eigen::Vector3f cameraForward = camRot.block<3, 3>(0, 0) * Eigen::Vector3f(0.f, 0.f, 1.f);
	Eigen::Vector3f cameraUp = camRot.block<3, 3>(0, 0) * Eigen::Vector3f(0.f, -1.f, 0.f);


	// *** Set up camera and output image ***
	Camera cam(
		cameraPos,
		cameraForward,
		cameraUp,
		pixWidth, pixHeight,
		config["cameraFov"]);


	std::vector<uint8_t> outImage(pixHeight * pixWidth * nChannels);

	Eigen::Vector3f
		red(1.f, 0.f, 0.f),
		blue(0.f, 0.f, 1.f),
		aqua(0.f, .8f, .8f),
		lavender(178.f / 255.f, 164.f / 255.f, 212.f / 255.f);

	std::vector<uint8_t> pABT;
	unsigned int pABTW, pABTH;
	lodepng::decode(pABT, pABTW, pABTH, "../models/tnwPowerArmourB.png");

	std::vector<uint8_t> pAHT;
	unsigned int pAHTW, pAHTH;
	lodepng::decode(pAHT, pAHTW, pAHTH, "../models/tnw_ncr_powerarmor_helmet.png");

	std::vector<uint8_t> pAGT;
	unsigned int pAGTW, pAGTH;
	lodepng::decode(pAGT, pAGTW, pAGTH, "../models/t45_paglove_ncr_d.png");

	std::vector<uint8_t> mgT;
	unsigned int mgTW, mgTH;
	lodepng::decode(mgT, mgTW, mgTH, "../models/minigun.png");

	std::vector<uint8_t> ncrFT;
	unsigned int ncrFTW, ncrFTH;
	lodepng::decode(ncrFT, ncrFTW, ncrFTH, "../models/nv_ncr_flag.png");

	std::vector<uint8_t> fPT;
	unsigned int fPTW, fPTH;
	lodepng::decode(fPT, fPTW, fPTH, "../models/nv_legionflag.png");

	std::vector<uint8_t> gT;
	unsigned int gTW, gTH;
	lodepng::decode(gT, gTW, gTH, "../models/dirtwasteland01.png");

	std::vector<uint8_t> sBT;
	unsigned int sBTW, sBTH;
	lodepng::decode(sBT, sBTW, sBTH, "../models/sandbag.png");

	std::vector<uint8_t> rT;
	unsigned int rTW, rTH;
	lodepng::decode(rT, rTW, rTH, "../models/roadwasteland01.png");

	std::vector<uint8_t> rHT;
	unsigned int rHTW, rHTH;
	lodepng::decode(rHT, rHTW, rHTH, "../models/ranchShack.png");

	std::vector<uint8_t> gSHT;
	unsigned int gSHTW, gSHTH;
	lodepng::decode(gSHT, gSHTW, gSHTH, "../models/goodShack.png");

	std::vector<uint8_t> sST;
	unsigned int sSTW, sSTH;
	lodepng::decode(sST, sSTW, sSTH, "../models/skyTex.png");

	std::vector<uint8_t> tBT;
	unsigned int tBTW, tBTH;
	lodepng::decode(tBT, tBTW, tBTH, "../models/Vurt_Jbark02.png");

	std::vector<uint8_t> tLT;
	unsigned int tLTW, tLTH;
	lodepng::decode(tLT, tLTW, tLTH, "../models/Vurt_JTreeTop42x.png");


	//All the Lambert Shaders
	TexturedLambertianShader pABTShader(&pABT, pABTW, pABTH);
	TexturedLambertianShader pAHTShader(&pAHT, pAHTW, pAHTH);
	TexturedLambertianShader pAGTShader(&pAGT, pAGTW, pAGTH);
	TexturedLambertianShader mgTShader(&mgT, mgTW, mgTH);
	TexturedLambertianShader ncrFTShader(&ncrFT, ncrFTW, ncrFTH);
	TexturedLambertianShader fPTShader(&fPT, fPTW, fPTH);
	TexturedLambertianShader gTShader(&gT, gTW, gTH);
	TexturedLambertianShader sBTShader(&sBT, sBTW, sBTH);
	TexturedLambertianShader rTShader(&rT, rTW, rTH);
	TexturedLambertianShader rHTShader(&rHT, rHTW, rHTH);
	TexturedLambertianShader gSHTShader(&gSHT, gSHTW, gSHTH);
	TexturedLambertianShader sSTShader(&sST, sSTW, sSTH);
	TexturedLambertianShader tBTShader(&tBT, tBTW, tBTH);
	TexturedLambertianShader tLTShader(&tLT, tLTW, tLTH);


	LambertianShader redLambertianShader(red);
	PhongShader bluePlasticShader(blue, Eigen::Vector3f(1.f, 1.f, 1.f), 100.f);
	LambertianShader aquaLambertianShader(aqua);
	LambertianShader lavenderLambertianShader(lavender);
	MirrorShader mirrorShader;
	TexCoordTestShader texCoordTestShader;

	// *** Set up scene ***
	Scene scene;

	// Optional code: here's how to add the spot mesh to the scene, using a BVH
	// Try enabling this and comparing it to the non-BVH version below!

	ModelSpawner builder{ scene };

	builder.addObject("../models/powerArmourBody.obj", &pABTShader, 4, { -1.0f, 5.0f,  25.f }, radians(180), radians(180), radians(0));
	builder.addObject("../models/powerArmourHelmet.obj", &pAHTShader, 4, { -1.0f, 2.65f, 39.f }, radians(180), radians(180), radians(0));
	builder.addObject("../models/powerArmourGloves.obj", &pAGTShader, 4, { 0.5f,  2.5f,  39.5f }, radians(180), radians(-185), radians(0));
	builder.addObject("../models/minigun.obj", &mgTShader, 4, { 8.75f,-4.f,   7.25f }, radians(-25), radians(150), radians(185));
	builder.addObject("../models/ncrFlagMesh2.obj", &ncrFTShader, 4, { -14.5f,12.f,  65.f }, radians(180), radians(25), radians(-10));
	builder.addObject("../models/ncrFlagPole.obj", &fPTShader, 4, { -20.f, 10.0f, 32.5f }, radians(180), radians(180), radians(0));
	builder.addObject("../models/groundMesh.obj", &fPTShader, 1, { 0.f,   25.f,  500.f }, radians(180), radians(0), radians(0));
	builder.addObject("../models/sandbagMesh.obj", &sBTShader, 4, { -30.f, 5.f,   45.f }, radians(180), radians(90), radians(0));
	builder.addObject("../models/sandbagMesh.obj", &sBTShader, 4, { -0.f,  7.5f,  110.f }, radians(180), radians(0), radians(0));
	builder.addObject("../models/roadMesh.obj", &rTShader, 4, { -135.f,2.0f,  100.f }, radians(175), radians(135), radians(0));
	builder.addObject("../models/ranchHouseM.obj", &rHTShader, 4, { -700.f,30.0f, 85.f }, radians(175), radians(135), radians(0));
	builder.addObject("../models/ranchHouseM.obj", &rHTShader, 4, { -625.f,10.0f, 65.f }, radians(175), radians(175), radians(0));
	builder.addObject("../models/goodspringHouseM.obj", &gSHTShader, 4, { 350.f, 24.0f, 50.f }, radians(174), radians(180), radians(0));
	builder.addObject("../models/skyMesh.obj", &sSTShader, 4, { 0.f,   100.0f,0.f }, radians(180), radians(180), radians(0));
	builder.addObject("../models/treeMesh.obj", &tBTShader, 4, { 47.5f, 5.f,   35.f }, radians(180), radians(180), radians(0));
	builder.addObject("../models/treeLeavesMesh.obj", &tLTShader, 4, { 47.5f, 5.f,   35.f }, radians(180), radians(180), radians(0));


	// *** Add lights to scene ***
	Eigen::Vector3f ambientLight(.8f, .8f, .8f);

	std::vector<std::unique_ptr<Light>> lightSources;
//	lightSources.push_back(std::make_unique<PointLight>(Eigen::Vector3f(-1.f, 3.f, -1.f), 3.f * Eigen::Vector3f(1.f, 1.f, 1.f)));
	lightSources.push_back(std::make_unique<DirectionalLight>(Eigen::Vector3f(0.f, -1.f, 1.f), 4.f * Eigen::Vector3f(1.f, 1.f, 1.f)));

	// *** Render the scene ***

	// Shuffling the scanline order gets better CPU usage between threads
	// when some lines take longer to render than others.
	std::vector<unsigned int> scanlines(pixHeight);
	for (int i = 0; i < pixHeight; ++i) scanlines[i] = i;

	if (config["shuffleScanlines"]) {
		std::random_device rd;
		std::mt19937 g(rd());
		std::shuffle(scanlines.begin(), scanlines.end(), g);
	}

	auto startTime = std::chrono::steady_clock::now();

	Ray ray = cam.getRay(531, 325);
	HitInfo hitInfo;
	scene.intersect(ray, 1e-6f, 1e6f, hitInfo, VISIBLE_BITMASK);
	float x = hitInfo.hitT;


	#pragma omp parallel for
	for (int y = 0; y < pixHeight; ++y) {
		for (int x = 0; x < pixWidth; ++x) {
			Ray ray = cam.getRay(x, scanlines[y]);
			HitInfo hitInfo;
			if (scene.intersect(ray, 1e-6f, 1e6f, hitInfo, VISIBLE_BITMASK)) {
				Eigen::Vector3f color = hitInfo.shader->getColor(
					hitInfo, &scene,
					lightSources, ambientLight,
					0, config["maxBounces"]);

				color.x() = std::min(color.x(), 1.f);
				color.y() = std::min(color.y(), 1.f);
				color.z() = std::min(color.z(), 1.f);


				int line = (pixHeight - scanlines[y]) - 1;
				outImage[(x + line * pixWidth) * nChannels + 0] = color.x() * 255;
				outImage[(x + line * pixWidth) * nChannels + 1] = color.y() * 255;
				outImage[(x + line * pixWidth) * nChannels + 2] = color.z() * 255;
				outImage[(x + line * pixWidth) * nChannels + 3] = 255;
			}
			else {
				int line = (pixHeight - scanlines[y]) - 1;
				outImage[(x + line * pixWidth) * nChannels + 0] = 0;
				outImage[(x + line * pixWidth) * nChannels + 1] = 0;
				outImage[(x + line * pixWidth) * nChannels + 2] = 0;
				outImage[(x + line * pixWidth) * nChannels + 3] = 255;
			}
		}
		if (omp_get_thread_num() == omp_get_num_threads()-1) {
			std::clog << "\rScanlines remaining: " << (pixHeight - y) << ' ' << std::flush;
		}

	}

	auto renderTime = std::chrono::steady_clock::now() - startTime;

	std::cout << "Render duration " << std::chrono::duration_cast<std::chrono::milliseconds>(renderTime).count() * 1e-3f << " seconds." << std::endl;

	// *** Save the output image ***
	int errorCode;
	errorCode = lodepng::encode(config["outputFilename"], outImage, pixWidth, pixHeight);
	if (errorCode) { // check the error code, in case an error occurred.
		std::cout << "lodepng error encoding image: " << lodepng_error_text(errorCode) << std::endl;
		return errorCode;
	}

	return 0;
}
