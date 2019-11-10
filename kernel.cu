
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <iostream>
#include <stdio.h>
#include <Windows.h>
#include "cuda_runtime.h"
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
#include <helper_cuda.h>
#include <curand_kernel.h>
#include <stdio.h>
#include <assert.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <map>

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
__global__ void fitness(float* commands, float* fitness, int nSteps, BoundaryConditions boundaryConditions,
	Weights weights)
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

}

__global__ void worstHalf(float* fitness, int* indexes)
{

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
		gpuErrchk(cudaMalloc((void**)& d_commands, 2 * 10 * populationSize * sizeof(float)));//commands
		float* h_commands = new float[2 * 10 * populationSize];
		for (int i = 0; i < 20 * populationSize; i++)
		{
			h_commands[i] = -0.44 + (float)(rand() % 88) / 100;
		}
		gpuErrchk(cudaMalloc((void**)& d_fitness, populationSize * sizeof(float)));
		cudaMemcpy(d_commands, h_commands, 20 * populationSize * sizeof(float), cudaMemcpyHostToDevice);

		fitness << <10, 100 >> > (d_commands, d_fitness, 10, boundaryConditions, weights);

		float* h_fitness = new float[populationSize];
		h_h = new float[populationSize];
		gpuErrchk(cudaMemcpy(h_fitness, d_fitness, populationSize * sizeof(float), cudaMemcpyDeviceToHost));
		for (int i = 0; i < populationSize * 20; i++)
		{
			//std::cout << h_fitness[i] << std::endl;
		}
		worstIndexes(h_fitness);
	}
	int* worstIndexes(float* fitness)
	{
		const int N = 6;
		int    keys[N] = { 1,   4,   2,   8,   5,   7 };
		char values[N] = { 'a', 'b', 'c', 'd', 'e', 'f' };
		thrust::sort_by_key(keys, keys + N, values);
		// keys is now   {  1,   2,   4,   5,   7,   8}
		// values is now {'a', 'c', 'b', 'e', 'f', 'd'}
		int* vals = new int[populationSize];
		int*  fit = new int[populationSize];
		for (int i = 0; i < populationSize; i++)
		{
			vals[i] = i;
			fit[i] = rand() % 20;
		}
		//for (int i = 0; i < populationSize; i++)
		//	printf("%f %d \n", fitness[i], vals[i]);// = i;
		//printf("--------------\n");
		//thrust::sort_by_key(fitness, fitness + populationSize, vals);
		//for (int i = 0; i < populationSize; i++)
		//	printf("%f %d \n",fitness[i], vals[i]);// = i;
		return NULL;
	}
	float* d_d;
	float* h_h;
	int populationSize = 1000;
	int numberOfSteps = 10;
	int numberOfGenerations = 100;
	float chanceOfMutation = 0.05;
	float chanceOfCrossover = 1;
	float* d_commands;
	float* d_fitness;
	BoundaryConditions boundaryConditions;
	Weights weights;
private:

};


int main()
{
	GpuTimer timer;
	timer.Start();
	Population ne;
	ne.initializePopulation();
	timer.Stop();
	printf("%f",timer.Elapsed());

	gpuErrchk(cudaGetLastError());
	gpuErrchk(cudaDeviceSynchronize());

	return 0;
}
