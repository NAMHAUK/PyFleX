class yz_BoxBath: public Scene
{
public:

	yz_BoxBath(const char* name, bool dam) : Scene(name), mDam(dam) {}

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

		int n_joint = 15;
		int n_geom  = 17;

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

		// 각 object를 이루는 particle들 첫 index와 끝 index 저장
		int particle_start_index[n_geom];
		int particle_end_index[n_geom];

		for(int i=0; i<n_geom; i++){
			particle_start_index[i] = g_buffers->positions.size();
			float pos_x = ptr[i*3];
			float pos_y = ptr[i*3 + 1];
			float pos_z = ptr[i*3 + 2];

			float scale_x = ptr[n_geom*3 + i*3];
			float scale_y = ptr[n_geom*3 + i*3 + 1];
			float scale_z = ptr[n_geom*3 + i*3 + 2];

			float mass = ptr[n_geom*3*2 + i] * 0.0001;

			CreateParticleShape(GetFilePathByPlatform(box_path).c_str(), Vec3(pos_x, pos_y, pos_z), Vec3(scale_x, scale_y, scale_z), 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), mass, true, 1.0f, NvFlexMakePhase(group, 0), false, 0.0f);
			particle_end_index[i] = g_buffers->positions.size()-1;
		}

		int total_particle_num = g_buffers->positions.size();
		float stiffness = 1.0f;
		
		int spring_count =0;
		int spring_jump = 20;

		// 각 자신보다 2칸까지 있는 object index
		
		int near_object_index_0[6] = {1,2,11,12,14,15};
		int near_object_index_1[7] = {0,2,3,4,8,11,14};
		int near_object_index_2[8] = {0,1,3,4,5,6,8,9};
		int near_object_index_3[5] = {1,2,4,5,8};
		int near_object_index_4[2] = {2,3};
		int near_object_index_5[6] = {1,2,3,6,7,8};
		int near_object_index_6[3] = {2,5,7};
		int near_object_index_7[2] = {5,6};
		int near_object_index_8[6] = {1,2,3,5,9,10};
		int near_object_index_9[3] = {2,8,10};
		int near_object_index_10[2] = {8,9};
		int near_object_index_11[5] = {0,1,12,13,14};
		int near_object_index_12[3] = {0,11,13};
		int near_object_index_13[2] = {11,12};
		int near_object_index_14[5] = {0,1,11,15,16};
		int near_object_index_15[3] = {0,14,16};
		int near_object_index_16[2] = {14,15};
		int* near_object_index[17] = {near_object_index_0, 
										near_object_index_1, 
										near_object_index_2, 
										near_object_index_3, 
										near_object_index_4, 
										near_object_index_5,
										near_object_index_6,
										near_object_index_7,
										near_object_index_8,
										near_object_index_9,
										near_object_index_10,
										near_object_index_11,
										near_object_index_12,
										near_object_index_13,
										near_object_index_14,
										near_object_index_15,
										near_object_index_16};
		for (int i=0; i<n_geom; i++){
			// 자신 근처 2칸까지 object들과 연결
			for(int j=0; j<(sizeof(near_object_index[i])/sizeof(int)); j++){
				int cur_near_object = near_object_index[i][j];

				// 자신과 다른 object spring으로 연결 (index는 spring_jump만큼 뛰면서)
				for(int l=particle_start_index[cur_near_object]; l<=particle_end_index[cur_near_object]; l+=spring_jump){
					for(int k=particle_start_index[i]; k<=particle_end_index[i]; k+=spring_jump){
						CreateSpring(l,k,stiffness);
						spring_count++;
					}
				}
			}
		}
		printf("spring number :%d",spring_count);
		// 자기 자신 object 말고 나머지랑 모두 연결
		// for(int i =0; i<n_geom; i++){
		// 	// 자신 보다 앞 object들과 모두 연결
		// 	for(int j=0; j<particle_start_index[i]; j+=spring_jump){
		// 		for(int k=particle_start_index[i]; k<=particle_end_index[i]; k+=spring_jump){
		// 			CreateSpring(j, k, stiffness);
		// 			spring_count++;
		// 		}
		// 	}
		// 	// 자신 보다 뒤 object들과 모두 연결
		// 	for(int j=particle_end_index[i]+1; j<total_particle_num; j+=spring_jump){
		// 		for(int k=particle_start_index[i]; k<=particle_end_index[i]; k+=spring_jump){
		// 			CreateSpring(j, k, stiffness);
		// 			spring_count++;
		// 		}
		// 	}
		// }
		// 
		// CreateSpring(spring[0],spring[1],stiffness);
		// CreateSpring(spring[1],spring[2],stiffness);

		// CreateSpring(spring[2],spring[3],stiffness);
		// CreateSpring(spring[2],spring[4],stiffness);

		// // head
		// CreateSpring(spring[2],spring[5],stiffness);

		// // right arm
		// CreateSpring(spring[3],spring[6],stiffness);
		// CreateSpring(spring[6],spring[7],stiffness);
		// CreateSpring(spring[7],spring[8],stiffness);
		
		// // left arm
		// CreateSpring(spring[4],spring[9],stiffness);
		// CreateSpring(spring[9],spring[10],stiffness);
		// CreateSpring(spring[10],spring[11],stiffness);

		// // right leg
		// CreateSpring(spring[0],spring[12],stiffness);
		// CreateSpring(spring[12],spring[13],stiffness);
		// CreateSpring(spring[13],spring[14],stiffness);

		// // left leg
		// CreateSpring(spring[0],spring[15],stiffness);
		// CreateSpring(spring[15],spring[16],stiffness);
		// CreateSpring(spring[16],spring[17],stiffness);


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

	bool mDam;
};
