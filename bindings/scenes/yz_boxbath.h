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

	// foot part spring 연결할 particle global index들
	int* get_box_points_foot(int start_index, int x, int y, int z){
		int n_x = (x-1)*0.5;
		int n_y = y*0.5;
		int n_z = (z-2)*0.5;

		int *arr = (int *)malloc(sizeof(int)*n_x*n_y*n_z);
		for(int i=0; i<n_x; i++){
			for(int j=0; j<n_y; j++){
				for(int k=0; k<3; k++){
					arr[n_z*n_y*i+ n_z*j + k] = start_index + z*y*(2*i+1) + z*j*2 + k*2+1;
				}
				for(int k=3; k<6; k++){
					arr[n_z*n_y*i+ n_z*j + k] = start_index + z*y*(2*i+1) + z*j*2 + k*2+2;
				}
			}
		}
		return arr;
	}

	// 높이 홀수 짝수인지 체크해서 3*3칸 마다 spring 연결할 particle global index들
	int* get_box_points(int start_index, int x, int y, int z){
		int type = 1;
		int n_x = (x-1)*0.5;
		int n_y = (y-1)*0.5;
		if(y%2 == 0){n_y = y*0.5; type = 0;}
		int n_z = (z-1)*0.5;

		int *arr = (int *)malloc(sizeof(int)*n_x*n_y*n_z);
		for(int i=0; i<n_x; i++){
			for(int j=0; j<n_y; j++){
				for(int k=0; k<n_z; k++){
					arr[n_z*n_y*i + n_z*j + k] = start_index + z*y*(2*i+1) + z*(2*j+type) + k*2+1;
				}
			}
		}
		return arr;
	}

	// 각 part의 spring 연결할 총 particle 개수를 구하는 함수
	int get_spring_num(int part_index, int x, int y, int z){
		// if foot
		if(part_index == 16 || part_index == 13){
			return (x-1)*0.5* y*0.5* (z-2)*0.5;
		}

		int n_x = (x-1)*0.5;
		int n_y = (y-1)*0.5;
		if(y%2 == 0){n_y = y*0.5;}
		int n_z = (z-1)*0.5;

		return n_x*n_y*n_z;
	}

	void Initialize(py::array_t<float> scene_params, int thread_idx = 0)
	{
		auto ptr = (float *) scene_params.request().ptr;

		int n_joint = 15;
		int n_geom  = 17;

		// particle parameters
		float radius = 0.1f;
		float s = radius*0.5f;
		float m = 0.25f;
		int group = 1;

		char box_path[100];
		make_path(box_path, "/data/box.ply");

		// spring parameters
		float stiffness = 1.0f;
		float connect_spring_stiffness = 3.0f;
		int spring_count =0;

		// 각 object를 이루는 particle들 첫 index와 끝 index 저장
		int particle_start_index[n_geom];
		int particle_end_index[n_geom];

		// 각 part를 unit_length로 나타낸 길이
		int part_scales[n_geom][3]  = {	{15, 15, 15},
                            		  	{11, 11, 11},
                            			{19+10, 19, 19},

                            			// neck
										{5, 5, 5},
										
										// head
										{15, 15, 15},

										// right arm
										{7, 23, 7},
										{5, 20, 5},
										{7, 7, 7},

										// left arm
										{7, 23, 7},
										{5, 20, 5},
										{7, 7, 7},

										// right leg
										{9, 34, 9},
										{7, 33, 7},
										{7, 4, 14},

										// left leg
										{9, 34, 9},
										{7, 33, 7},
										{7, 4, 14}
		};

		// ------------------------------------------ 각 part 생성 -----------------------------------------
		for(int i=0; i<n_geom; i++){
			particle_start_index[i] = g_buffers->positions.size();
			float pos_x = ptr[i*3];
			float pos_y = ptr[i*3 + 1];
			float pos_z = ptr[i*3 + 2];

			float scale_x = ptr[n_geom*3 + i*3];
			float scale_y = ptr[n_geom*3 + i*3 + 1];
			float scale_z = ptr[n_geom*3 + i*3 + 2];

			float invmass = (part_scales[i][0]*part_scales[i][1]*part_scales[i][2]) / ptr[n_geom*3*2 + i] ;
			printf("invmass %d : %f\n",i,invmass);

			CreateParticleShape(GetFilePathByPlatform(box_path).c_str(), Vec3(pos_x, pos_y, pos_z), Vec3(scale_x, scale_y, scale_z), 0.0f, s, Vec3(0.0f, 0.0f, 0.0f), invmass, true, 1.0f, NvFlexMakePhase(group, 0), false, 0.0f);
			particle_end_index[i] = g_buffers->positions.size()-1;
		}
		printf("particle num : %d \n", g_buffers->positions.size());
	
		// --------------------- 같은 joint인 부분 상대 위치 안 변하게 spring으로 묶기(붙이기) --------------------
		// {몸통 1, 몸통 2}
		for(int i=0; i<11; i++){
			for(int j=0; j<11; j++){
				int index1 = particle_start_index[0]+ 15*15*(2+i) + 15*13 + j+2;
				int index2 = particle_start_index[1]+ 11*11*i + 11*1 + j;
				CreateSpring(index1, index2, stiffness);
			}
		}
		// {머리, 목}
		for(int i=0; i<5; i++){
			for(int j=0; j<5; j++){
				int index1 = particle_start_index[3]+ 5*5*i + 5*4 + j;
				int index2 = particle_start_index[4]+ 15*15*(5+i) + j+5;
				CreateSpring(index1, index2, stiffness);
			}
		}
		
		// --------------------- length 0인 각 part 연결부위(particle) spring으로 연결 ---------------------
		
		// 연결 부위 particle 쌍의 global index
		int connect_particle_index[14][2] ={
											{particle_start_index[1]+ 11*11*5 + 11*9 + 5  , particle_start_index[2]+ 19*19*14 + 9},			// body2 & body3
											{particle_start_index[2]+ 19*19*14 + 19*18 + 9, particle_start_index[3]+ 5*5*2 + 2},			// body3 & neck

											// right arm
											{particle_start_index[2]+ 19*19*28 + 19*18 + 9, particle_start_index[5]+ 7*23*3 + 7*22 + 3},	// body3 & U_arm
											{particle_start_index[5]+ 7*23*3 + 3          , particle_start_index[6]+ 5*20*2 + 5*19 + 2},	// U_arm & L_arm
											{particle_start_index[6]+ 5*20*2 + 2  		  , particle_start_index[7]+ 7*7*3 + 7*4 + 3},		// L_arm & hand

											// left arm
											{particle_start_index[2]+ 19*18 + 9			  , particle_start_index[8]+ 7*23*3 + 7*22 + 3},	// body3 & U_arm
											{particle_start_index[8]+ 7*23*3 + 3          , particle_start_index[9]+ 5*20*2 + 5*19 + 2},	// U_arm & L_arm
											{particle_start_index[9]+ 5*20*2 + 2  		  , particle_start_index[10]+ 7*7*3 + 7*4 + 3},		// L_arm & hand

											// right leg
											{particle_start_index[0]+ 15*15*7 + 15*1 + 7  , particle_start_index[11]+ 9*34*4 + 9*33 + 4},	// body1 & U_leg
											{particle_start_index[11]+ 9*34*4 + 4         , particle_start_index[12]+ 7*33*3 + 7*32 + 3},	// U_leg & L_leg
											{particle_start_index[12]+ 7*33*3 + 3  		  , particle_start_index[13]+ 14*4*3 + 14*3 + 3},	// L_leg & foot

											// left leg
											{particle_start_index[0]+ 15*1 + 7  		  , particle_start_index[14]+ 9*34*4 + 9*33 + 4},	// body1 & U_leg
											{particle_start_index[14]+ 9*34*4 + 4         , particle_start_index[15]+ 7*33*3 + 7*32 + 3},	// U_leg & L_leg
											{particle_start_index[15]+ 7*33*3 + 3  		  , particle_start_index[16]+ 14*4*3 + 14*3 + 3},	// L_leg & foot
		};

		// 연결 부위 particle 쌍 spring으로 rest_length 0으로 묶음
		for (int i=0 ; i<14; i++){
			CreateSpring(connect_particle_index[i][0], connect_particle_index[i][1], connect_spring_stiffness);
		}
		
		// --------------------- 캐릭터 동작을 위해 각 part spring으로 연결 ---------------------

		// 각 part 내부 일정 거리마다 있는 particle들 spring 연결
		// 각 part에서 spring으로 연결할 particle들 global index 계산하여 저장
		int ** all_box_points = (int **) malloc ( sizeof(int *) * n_geom);
		for (int i=0; i<n_geom ; i++){

			// if foot
			if(i==n_geom-1-3 || i==n_geom-1){
				all_box_points[i] = get_box_points_foot(particle_start_index[i], part_scales[i][0], part_scales[i][1], part_scales[i][2]);
				continue;
			}

			// other parts
			all_box_points[i] = get_box_points(particle_start_index[i], part_scales[i][0], part_scales[i][1], part_scales[i][2]);
		}

		// 각 자신보다 2칸까지 있는 near object index
		int near_object_index_0[6] = {1,2,11,12,14,15};
		int near_object_index_1[7] = {2,3,4,8,11,14};
		int near_object_index_2[8] = {3,4,5,6,8,9};
		int near_object_index_3[5] = {4,5,8};
		int near_object_index_4[2] = {};
		int near_object_index_5[6] = {6,7,8};
		int near_object_index_6[3] = {7};
		int near_object_index_7[2] = {};
		int near_object_index_8[6] = {9,10};
		int near_object_index_9[3] = {10};
		int near_object_index_10[2] = {};
		int near_object_index_11[5] = {12,13,14};
		int near_object_index_12[3] = {13};
		int near_object_index_13[2] = {};
		int near_object_index_14[5] = {15,16};
		int near_object_index_15[3] = {16};
		int near_object_index_16[2] = {};
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
		
		int near_object_index_len[17]= {6,6,6,3,0,3,1,0,2,1,0,3,1,0,2,1,0};
		// near part끼리만 spring 연결
		for (int i=0; i<n_geom; i++){
			for(int n=0; n<near_object_index_len[i]; n++){

				// 근처 object index
				int j = near_object_index[i][n];
				// 각 part의 spring 연결할 particle 개수
				int i_size = get_spring_num(i, part_scales[i][0], part_scales[i][1], part_scales[i][2]);
				int j_size = get_spring_num(j, part_scales[j][0], part_scales[j][1], part_scales[j][2]);

				// 각 part의 spring 연결할 particle끼리 spring 연결
				for(int a=0; a<i_size; a++){
					for(int b=0; b<j_size; b++){
						CreateSpring(all_box_points[i][a], all_box_points[j][b], stiffness);
						spring_count ++;
					}
				}
			}
		}
		printf("spring number :%d\n",spring_count);

		//--------------------- setting scene parameters ---------------------

		// spring을 화면에 보이게 할 것인지
		float draw_spring = ptr[n_geom*7];
		g_drawSprings = true;
		if(draw_spring == 0.0){
			printf("don't draw Spring");
			g_drawSprings = false;
		}

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