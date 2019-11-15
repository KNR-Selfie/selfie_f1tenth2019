
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string>
#include "device_launch_parameters.h"
#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <iostream>
#include <stdio.h>
#include <thrust/device_vector.h>
#include "gputimer.h"
#include <thrust/transform.h>
#include <thrust/execution_policy.h>
#include <thrust/sequence.h>
#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/replace.h>
#include <thrust/functional.h>
#include <iostream>
#include <thrust/sort.h>
#include <curand.h>
#include <math.h>
#include <curand.h>
#include <device_functions.h>
#include "helper_cuda.h"
#include <curand_kernel.h>
#include <stdio.h>
#include <assert.h>
#include <string>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true)
{
	if (code != cudaSuccess)
	{
		fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
		if (abort) exit(code);
	}
}
struct BoundaryConditions
{
	float x0 = 1;
	float y0 = 1;
	float psi0 = 1;
	float cte0 = 1;
	float e0 = 1;
	float l = 1;
	float a0 = 1;
	float a1 = 1;
	float a2 = 1;
	float dt = 0.1;
};
struct Weights
{
	float wCTE = 1;
	float wE = 1;
	float wO = 1;
	float wV = 1;
	float wDo = 1;
	float wDv = 1;
	float wC = 1;
	float Vmax = 1;
};
__global__ void fitness(float* commands, float* fitness, int nSteps, BoundaryConditions boundaryConditions, Weights weights)
{
	int k = threadIdx.x + blockDim.x * blockIdx.x;

	//Initial conditions
	float x = boundaryConditions.x0;
	float y = boundaryConditions.y0;
	float psi = boundaryConditions.psi0;
	float cte = boundaryConditions.cte0;
	float e = boundaryConditions.e0;
	float a2 = boundaryConditions.a2;
	float a1 = boundaryConditions.a1;
	float a0 = boundaryConditions.a0;
	float dt = boundaryConditions.dt;
	float l = boundaryConditions.l;

	float cost = 0;

	//derrivatives
	float xD = 0;
	float yD = 0;
	float psiD = 0;
	float lastAngle = 0;
	float lastSpeed = 0;

	for (int i = 0; i < nSteps; i++)
	{
		int specimenOffset = nSteps * 2 * k;
		float speed = commands[specimenOffset + 2 * i];
		float angle = commands[specimenOffset + 2 * i + 1];

		//error
		cte = a2 * x * x + a1 * x + a0 - y;
		e = atan(2 * a2 * x + a1);

		//state update
		xD = speed * cos(psi);
		yD = speed * sin(psi);
		psiD = speed * angle / l;
		x += xD * dt;
		y += yD * dt;
		psi += psiD * dt;

		cost +=
			weights.wCTE * pow(cte, 2) +
			weights.wE * pow(e, 2) +
			weights.wO * pow(angle, 2) +
			weights.wV * pow(weights.Vmax - speed, 2) +
			weights.wDo * pow(angle - lastAngle, 2) +
			weights.wDv * pow(speed - lastSpeed, 2) +
			weights.wC * pow(speed * angle, 2);

		lastAngle = angle;
		lastSpeed = speed;
	}
	fitness[k] = cost;
#ifdef DEBUGCOST
	printf("Koszt wynosi: %f\n", fitness[k]);
#endif // DEBUG

}
__global__ void sortWithIndexes(float* fitness, int* indexes, int populationSize)//dziala
{
#ifdef DEBUGSORT
	for (int i = 0; i < populationSize; i++)
		printf("Indeks i fitness przed sortowaniem: %d %f\n", indexes[i], fitness[i]);
#endif // DEBUGSORT

	thrust::sort_by_key(thrust::device, fitness, fitness + populationSize, indexes);
#ifdef DEBUGSORT
	for (int i = 0; i < populationSize; i++)
		printf("Indeks i fitness po sortowaniu: %d %f\n", indexes[i], fitness[i]);
#endif // DEBUGSORT
}
__global__ void printCommands(float* commands)
{
	int k = threadIdx.x + blockDim.x * blockIdx.x;
	for (int i = 0; i < 20; i++)
	{
		if ((k * 20 + i) % 2 == 0)
			printf("skret %f\n", commands[k * 20 + i]);
		else
			printf("predkosc %f\n", commands[k * 20 + i]);
	}
}
__global__ void crossover(float* commands, int* indexes, int* random)
{
	int k = threadIdx.x + blockDim.x * blockIdx.x;
	for (int i = 0; i < 10; i++)
	{
		int rand1 = random[k]; __syncthreads();
		int rand2 = random[500 + k]; __syncthreads(); //random[0,500]
		int parentOffset1 = 20 * indexes[rand1]; __syncthreads();//k jest z zakresu 0-50%pop(najlepsza cz�� populacji)
		int parentOffset2 = 20 * indexes[rand2]; __syncthreads();//k jest z zakresu 0-50%pop(najlepsza cz�� populacji)
		if (i < (k % 10))
		{
			float speed = commands[parentOffset1 + 2 * i + 1];
			__syncthreads();
			float angle = commands[parentOffset1 + 2 * i]; __syncthreads();
			//printf("%d \n", 500 + k * 20 + 2 * i + 1);
			commands[(500 + k) * 20 + 2 * i + 1] = speed; __syncthreads();
			commands[(500 + k) * 20 + 2 * i] = angle; __syncthreads();
		}
		else
		{
			float speed = commands[parentOffset2 + 2 * i + 1]; __syncthreads();
			float angle = commands[parentOffset2 + 2 * i]; __syncthreads();
			//printf("%d \n", 500 + k * 20 + 2 * i + 1);
			commands[(500 + k) * 20 + 2 * i + 1] = speed; __syncthreads();
			commands[(500 + k) * 20 + 2 * i] = angle; __syncthreads();
		}
	};
}
int* generateRandom(int min, int max, int size)
{
	srand(time(NULL));
	int* random = new int[size];
	for (int i = 0; i < size; i++)
	{
		random[i] = min + rand() % max;
	}
	return random;
}


class Population
{
public:
	Population(int populationSize, int numberOfSteps, int numberOfGenerations, float chanceOfMutation, float chanceOfCrossover)
	{
		this->populationSize = populationSize;
		this->numberOfSteps = numberOfSteps;
		this->numberOfGenerations = numberOfGenerations;
		this->chanceOfMutation = chanceOfMutation;
		this->chanceOfCrossover = chanceOfCrossover;
	}
	Population()
	{
		this->populationSize = 1000;
		this->numberOfSteps = 10;
		this->numberOfGenerations = 100;
		this->chanceOfMutation = 0.05;
		this->chanceOfCrossover = 1;

	}
	void initializePopulation()
	{
		srand(time(NULL));

		int* vals_temp = new int[populationSize];
		h_commands = new float[2 * 10 * populationSize];
		for (int i = 0; i < populationSize; i++)
		{
			vals_temp[i] = i;
		}
		for (int i = 0; i < 20 * populationSize; i++)
		{
			if (i % 2 == 0)
				h_commands[i] = -0.44 + (float)(rand() % 88) / 100;
			else
				h_commands[i] = -2 + (float)(rand() % 400) / 100;
		}

		gpuErrchk(cudaMalloc((void**)& d_vals, populationSize * sizeof(int)));
		gpuErrchk(cudaMalloc((void**)& d_commands, 2 * 10 * populationSize * sizeof(float)));//commands
		gpuErrchk(cudaMalloc((void**)& d_fitness, populationSize * sizeof(float)));

		gpuErrchk(cudaMemcpy(d_vals, vals_temp, populationSize * sizeof(int), cudaMemcpyHostToDevice));
		gpuErrchk(cudaMemcpy(d_commands, h_commands, 20 * populationSize * sizeof(float), cudaMemcpyHostToDevice));
#ifdef DEBUGCOMMANDS
		for (int i = 0; i < 20 * populationSize - 1; i++)
		{
			printf("%f;%f\n", h_commands[i], h_commands[i + 1]);
		}
#endif // DEBUGCOMMANDS
	}
	void dumpLog(std::ofstream& plik)
	{
		for (int j = 0; j < 10; j++)
			plik << "angle;";
		plik << "NULL;";
		for (int j = 0; j < 10; j++)
			plik << "speed;";
		plik << "\n";
		for (int i = 0; i < populationSize; i++)
		{
			plik << "=" << i << ";";
			for (int j = 0; j < 10; j++)
				plik << "=" << h_commands[i * 20 + j * 2] << ";";
			plik << "NULL;";
			for (int j = 0; j < 10; j++)
				plik << "=" << h_commands[i * 20 + j * 2 + 1] << ";";
			plik << "\n";
		}
	}
	void geneticAlgorithm()
	{

		int* randomNumbers = new int[1000];
		randomNumbers = generateRandom(0, 500, 1000);
		int* d_randomNumbers;
		cudaMalloc((void**)& d_randomNumbers, 1000 * sizeof(int));
		cudaMemcpy(d_randomNumbers, randomNumbers, 1000 * sizeof(int), cudaMemcpyHostToDevice);
		fitness << <10, 100 >> > (d_commands, d_fitness, 10, boundaryConditions, weights);
		sortWithIndexes << <1, 1 >> > (d_commands, d_vals, populationSize);


#ifdef DUMPLOG
		gpuErrchk(cudaMemcpy(h_commands, d_commands, 10 * 2 * populationSize * sizeof(float), cudaMemcpyDeviceToHost));
		std::ofstream plik("dzial.csv");
		dumpLog(plik);
#endif // DUMPLOG
		crossover << <10, 50 >> > (d_commands, d_vals, d_randomNumbers);
#ifdef DUMPLOG
		gpuErrchk(cudaMemcpy(h_commands, d_commands, 10 * 2 * populationSize * sizeof(float), cudaMemcpyDeviceToHost));
		dumpLog(plik);
#endif // DUMPLOG

	}
	float* d_d;
	float* h_h;
	float* h_commands;
	int populationSize = 1000;
	int numberOfSteps = 10;
	int numberOfGenerations = 100;
	float chanceOfMutation = 0.05;
	float chanceOfCrossover = 1;
	float* d_commands;
	float* d_fitness;
	int* d_vals;
	BoundaryConditions boundaryConditions;
	Weights weights;
private:

};

std_msgs::Float64 steering_angle_msg;
std_msgs::Float64 target_speed_msg;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "genetic_node");
	ros::NodeHandle nh;
	ros::Publisher steering_angle = nh.advertise<std_msgs::Float64>("steering_angle", 1000);
	ros::Publisher target_speed = nh.advertise<std_msgs::Float64>("target_speed", 1000);
	//
	GpuTimer timer;
	Population ne;
	timer.Start();
	ne.initializePopulation();
	timer.Stop();
	printf("%f\n", timer.Elapsed());
	timer.Start();
	for (int i = 0; i < 1; i++)
		ne.geneticAlgorithm();
	timer.Stop();
	printf("%f\n", timer.Elapsed());
	gpuErrchk(cudaGetLastError());
	gpuErrchk(cudaDeviceSynchronize());
	//
	ros::Rate rate(10);
	while(ros::ok())
	{
		steering_angle_msg.data = 21.37; //update komend skretu here
		steering_angle.publish(steering_angle_msg);
		target_speed_msg.data = 997.00;
		target_speed.publish(target_speed_msg);

		ros::spinOnce();
    rate.sleep();
	}
	return 0;
}
