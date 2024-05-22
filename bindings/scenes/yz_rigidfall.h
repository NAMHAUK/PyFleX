
class yz_RigidFall: public Scene
{
public:

	yz_RigidFall(const char* name) : Scene(name) {}

	char* make_path(char* full_path, std::string path) {
		strcpy(full_path, getenv("PYFLEXROOT"));
		strcat(full_path, path.c_str());
		return full_path;
	}

	float rand_float(float LO, float HI) {
        return LO + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(HI-LO)));
    }

    void swap(float* a, float* b) {
	    float tmp = *a;
	    *a = *b;
	    *b = tmp;
	}

	void Initialize(py::array_t<float> scene_params, int thread_idx = 0)
	{
		auto ptr = (float *) scene_params.request().ptr;
		int n_instance = (int) ptr[0];

		float radius = 0.1f;

		// deforming bunny
		float s = radius*0.5f;
		float m = 0.25f;
		int group = 1;

		char bunny_path[100];
		char box_path[100];
		char sphere_path[100];
		make_path(bunny_path, "/data/bunny.ply");
		make_path(box_path, "/data/box.ply");
		make_path(sphere_path, "/data/sphere.ply");

		// 밑 판 param
		float x = ptr[1];
		float y = ptr[2];
		float z = ptr[3];

		// 스틱 param
		float x2 = ptr[4];
		float y2 = ptr[5];
		float z2 = ptr[6];

		// 밑 판 만들기 (5 x 5 크기)
		CreateParticleShape(GetFilePathByPlatform(box_path).c_str(), Vec3(x, y, z), Vec3(0.25f,0.05f,0.25f), 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, NvFlexMakePhase(group, 0), false, 0.0f);
		
		// 스틱 만들기 (1 x 5 크기)
		CreateParticleShape(GetFilePathByPlatform(box_path).c_str(), Vec3(x2, y2, z2), Vec3(0.05f,0.25f,0.05f), 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, NvFlexMakePhase(group++, 0), false, 0.0f);
		int particle_num = g_buffers->positions.size();

		// printf("particle create 완료\n");

		float center_stiffness = ptr[7];
		float line_stiffness   = ptr[8];

		int stick_bottion_index = 25;
		int stick_top_index     = 29;
		// printf("particle num : %d\n", particle_num);
		
		// 막대 밑이랑 판 중간이랑 묶음
		CreateSpring(6, stick_bottion_index, center_stiffness);
		CreateSpring(7, stick_bottion_index, center_stiffness);
		CreateSpring(8, stick_bottion_index, center_stiffness);

		CreateSpring(11, stick_bottion_index, center_stiffness);
		CreateSpring(12, stick_bottion_index, center_stiffness);
		CreateSpring(13, stick_bottion_index, center_stiffness);

		CreateSpring(16, stick_bottion_index, center_stiffness);
		CreateSpring(17, stick_bottion_index, center_stiffness);
		CreateSpring(18, stick_bottion_index, center_stiffness);

		//막대 끝이랑 묶음
		int left_front_index  = 0;
		int left_back_index   = 4;
		int right_front_index = 20;
		int right_back_index  = 24;

		CreateSpring(0, stick_top_index, line_stiffness);
		CreateSpring(2, stick_top_index, line_stiffness);
		CreateSpring(4, stick_top_index, line_stiffness);
		CreateSpring(14, stick_top_index, line_stiffness);
		CreateSpring(24, stick_top_index, line_stiffness);
		CreateSpring(22, stick_top_index, line_stiffness);
		CreateSpring(20, stick_top_index, line_stiffness);
		CreateSpring(10, stick_top_index, line_stiffness);
		
		// printf("%d spring 생성 완료 \n", g_buffers->springLengths.size());

		// for (int i = 0;i< g_buffers->springLengths.size();i++){
		// 	printf("%d : %f\n ",i, g_buffers->springLengths[i]);
		// }

		g_numSolidParticles = g_buffers->positions.size();

		float restDistance = radius*0.55f;

		g_sceneLower = Vec3(0.0f, 0.0f, 0.0f);
		g_sceneUpper = Vec3(0.6f, 0.0f, 0.4f);

		g_numSubsteps = 2;

		g_params.radius = radius;
		g_params.dynamicFriction = 1.0f;
		g_params.viscosity = 2.0f;
		g_params.numIterations = 4;
		g_params.vorticityConfinement = 40.0f;
		g_params.fluidRestDistance = restDistance;
		g_params.solidPressure = 0.f;
		g_params.relaxationFactor = 0.0f;
		g_params.cohesion = 0.02f;
		g_params.collisionDistance = 0.01f;		

		g_maxDiffuseParticles = 0;
		g_diffuseScale = 0.5f;

		g_fluidColor = Vec4(0.113f, 0.425f, 0.55f, 1.f);

		Emitter e1;
		e1.mDir = Vec3(1.0f, 0.0f, 0.0f);
		e1.mRight = Vec3(0.0f, 0.0f, -1.0f);
		e1.mPos = Vec3(radius, 1.f, 0.65f);
		e1.mSpeed = (restDistance/g_dt)*2.0f; // 2 particle layers per-frame
		e1.mEnabled = true;

		g_emitters.push_back(e1);

		// g_numExtraParticles = 48*1024;

		g_lightDistance = 1.8f;

		// g_params.numPlanes = 5;

		g_waveFloorTilt = 0.0f;
		g_waveFrequency = 1.5f;
		g_waveAmplitude = 2.0f;
		
		g_warmup = false;

		// draw options		
		g_drawPoints = true;
		g_drawMesh = false;
		g_drawEllipsoids = false;
		g_drawDiffuse = true;
	}
	void Update(){
		printf("okayayyy");
	}
};

// class yz_RigidFall: public Scene
// {
// public:

// 	yz_RigidFall(const char* name) : Scene(name) {}

// 	char* make_path(char* full_path, std::string path) {
// 		strcpy(full_path, getenv("PYFLEXROOT"));
// 		strcat(full_path, path.c_str());
// 		return full_path;
// 	}

// 	float rand_float(float LO, float HI) {
//         return LO + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/(HI-LO)));
//     }

//     void swap(float* a, float* b) {
// 	    float tmp = *a;
// 	    *a = *b;
// 	    *b = tmp;
// 	}

// 	void Initialize(py::array_t<float> scene_params, int thread_idx = 0)
// 	{
// 		auto ptr = (float *) scene_params.request().ptr;
// 		int n_instance = (int) ptr[0];

// 		float radius = 0.1f;

// 		// deforming bunny
// 		float s = radius*0.5f;
// 		float m = 0.25f;
// 		int group = 1;

// 		char bunny_path[100];
// 		char box_path[100];
// 		char sphere_path[100];
// 		make_path(bunny_path, "/data/bunny.ply");
// 		make_path(box_path, "/data/box.ply");
// 		make_path(sphere_path, "/data/sphere.ply");

// 		float x = ptr[1];
// 		float y = ptr[2];
// 		float z = ptr[3];

// 		float x2 = ptr[1];
// 		float y2 = ptr[2];
// 		float z2 = ptr[3];

// 		CreateParticleShape(GetFilePathByPlatform(box_path).c_str(), Vec3(x, y, z), Vec3(0.25f,0.1f,0.25f), 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, NvFlexMakePhase(group++, 0), false, 0.0f);
// 		int box_particle_num = g_buffers->positions.size();
// 		CreateParticleShape(GetFilePathByPlatform(box_path).c_str(), Vec3(x2, y2, z2), Vec3(0.15f,0.3f,0.15f), 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), m, true, 1.0f, NvFlexMakePhase(group++, 0), false, 0.0f);
		
		
// 		printf("AA");
// 		int box_middle = 27;

// 		float stiffness = ptr[7];

// 		int stick_top1 = ptr[8];
// 		int stick_top2 = ptr[9];
// 		int stick_top3 = ptr[10];
// 		int stick_top4 = ptr[11];
// 		printf("%d %d", box_particle_num, box_middle);
		
// 		// 막대 밑이랑 판 중간이랑 묶음
// 		CreateSpring(16, box_particle_num    , 1.0f);
// 		CreateSpring(17, box_particle_num + 1, 1.0f);
// 		CreateSpring(18, box_particle_num + 2, 1.0f);

// 		CreateSpring(26, box_particle_num + 18, 1.0f);
// 		CreateSpring(27, box_particle_num + 19, 1.0f);
// 		CreateSpring(28, box_particle_num + 20, 1.0f);

// 		CreateSpring(36, box_particle_num + 36, 1.0f);
// 		CreateSpring(37, box_particle_num + 37, 1.0f);
// 		CreateSpring(38, box_particle_num + 38, 1.0f);

// 		//막대 끝이랑 묶음
// 		CreateSpring(stick_top1, 65, stiffness);
// 		CreateSpring(stick_top2, 67, stiffness);
// 		CreateSpring(stick_top3, 101, stiffness);
// 		CreateSpring(stick_top4, 103, stiffness);
// 		printf("BB");

// 		g_numSolidParticles = g_buffers->positions.size();

// 		float restDistance = radius*0.55f;

// 		g_sceneLower = Vec3(0.0f, 0.0f, 0.0f);
// 		g_sceneUpper = Vec3(0.6f, 0.0f, 0.4f);

// 		g_numSubsteps = 2;

// 		g_params.radius = radius;
// 		g_params.dynamicFriction = 1.0f;
// 		g_params.viscosity = 2.0f;
// 		g_params.numIterations = 4;
// 		g_params.vorticityConfinement = 40.0f;
// 		g_params.fluidRestDistance = restDistance;
// 		g_params.solidPressure = 0.f;
// 		g_params.relaxationFactor = 0.0f;
// 		g_params.cohesion = 0.02f;
// 		g_params.collisionDistance = 0.01f;		

// 		g_maxDiffuseParticles = 0;
// 		g_diffuseScale = 0.5f;

// 		g_fluidColor = Vec4(0.113f, 0.425f, 0.55f, 1.f);

// 		Emitter e1;
// 		e1.mDir = Vec3(1.0f, 0.0f, 0.0f);
// 		e1.mRight = Vec3(0.0f, 0.0f, -1.0f);
// 		e1.mPos = Vec3(radius, 1.f, 0.65f);
// 		e1.mSpeed = (restDistance/g_dt)*2.0f; // 2 particle layers per-frame
// 		e1.mEnabled = true;

// 		g_emitters.push_back(e1);

// 		// g_numExtraParticles = 48*1024;

// 		g_lightDistance = 1.8f;

// 		// g_params.numPlanes = 5;

// 		g_waveFloorTilt = 0.0f;
// 		g_waveFrequency = 1.5f;
// 		g_waveAmplitude = 2.0f;
		
// 		g_warmup = false;

// 		// draw options		
// 		g_drawPoints = true;
// 		g_drawMesh = false;
// 		g_drawEllipsoids = false;
// 		g_drawDiffuse = true;
// 	}
// 	void Update(){
// 		printf("okayayyy");
// 	}
// };
