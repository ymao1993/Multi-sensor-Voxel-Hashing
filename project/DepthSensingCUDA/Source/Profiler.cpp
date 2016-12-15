#include "stdafx.h"
#include "Profiler.h"
#include <iostream>
#include <algorithm>
#include "cudaUtil.h"

#define EXIT(STATUS)  {system("pause"); exit(STATUS);}

using namespace std;

void Profiler::startTiming(const string& token, int startFrame)
{
	cudaDeviceSynchronize();
	if (timingLogs.find(token) == timingLogs.end()) {
		timingLogs[token] = TimingLog(token);
	}

	TimingLog& log = timingLogs[token];
	if (log.isTiming) {
		cout << "[startTiming] Error: the profiler is still timing another session for [" << token.c_str() << "]." << endl;
		EXIT(-1);
	}

	log.isTiming = true;
	log.start_frames.push_back(startFrame);

	LARGE_INTEGER li;
	QueryPerformanceCounter(&li);
	log.start = li.QuadPart;
}

void Profiler::stopTiming(const string& token, int endFrame)
{
	cudaDeviceSynchronize();
	while (true)
	{
		if (timingLogs.find(token) == timingLogs.end()) break;
		TimingLog& log = timingLogs[token];
		if (!log.isTiming) break;

		LARGE_INTEGER li;
		QueryPerformanceCounter(&li);
		log.end = li.QuadPart;
		QueryPerformanceFrequency(&li);
		double frequency = double(li.QuadPart) / 1000.0;
		double elapsed = double(log.end - log.start) / frequency;

		log.elapsed_time.push_back(elapsed);
		log.isTiming = false;
		log.end_frames.push_back(endFrame);
		return;
	}

	cout << "[stopTiming] Error: no active timing session for [" << token.c_str() << "] found." << endl;
	EXIT(-1);
	return;
}

void Profiler::printTimingLog(const string& token)
{
	if (timingLogs.find(token) == timingLogs.end()) {
		cout << "[printTimingLog] Error: no matching logs for [" << token.c_str() << "] found." << endl;
		EXIT(-1);
	}
	TimingLog& log = timingLogs[token];
	vector<double>& elapsed_time = log.elapsed_time;
	for (int i = 0; i < elapsed_time.size(); i++)
	{
		std::cout << i + 1 << ": " << elapsed_time[i] << " ms" << std::endl;
	}
}

bool Profiler::TimingStatsEntryCompare(Profiler::TimingStatsEntry& lhs, Profiler::TimingStatsEntry& rhs) {
	return lhs.totalTime > rhs.totalTime;
}

void Profiler::generateTimingStats()
{
	timingEntries.clear();
	for (auto it = timingLogs.begin(); it != timingLogs.end(); it++)
	{
		TimingStatsEntry entry(it->first);
		vector<double> &elapsed_time = it->second.elapsed_time;
		for (int i = 0; i < elapsed_time.size(); i++)
		{
			entry.totalTime += elapsed_time[i];
		}
		entry.averageTime = ((double)entry.totalTime) / elapsed_time.size();
		entry.length = elapsed_time.size();
		timingEntries.push_back(entry);
	}
	std::sort(timingEntries.begin(), timingEntries.end(), TimingStatsEntryCompare);
	isTimingStatsGenerated = true;
}

void Profiler::printTimingStats()
{
	printf("=================Time Stats=================\n");
	printf("id\tbucket name\ttotal time\taverage time\tlength\n");
	for (int i = 0; i < timingEntries.size(); i++)
		printf("[%d]\t%s\t%.2lfms\t%.4lfms\t%d\n", i + 1, timingEntries[i].token.c_str(), timingEntries[i].totalTime, timingEntries[i].averageTime, timingEntries[i].length);
	printf("===================END======================\n");
	return;
}

void Profiler::recordDataPoints(const std::string& token, float y, float x /* = 0 */)
{
	if (dataPointsLogs.find(token) == dataPointsLogs.end()) {
		DataPointsLog log(token);
		dataPointsLogs[token] = log;
	}
	dataPointsLogs[token].X.push_back(x);
	dataPointsLogs[token].Y.push_back(x);
}

void Profiler::resetTiming()
{
	timingEntries.clear();
	timingLogs.clear();
	isTimingStatsGenerated = false;
}

void Profiler::resetDataPointsRecording()
{
	dataPointsLogs.clear();
}

void Profiler::resetAll()
{
	resetTiming();
	resetDataPointsRecording();
}

void Profiler::dumpToFolderAll(const std::string& folderPath)
{
	dumpToFolderTimingLogs(folderPath);
	dumpToFolderDataPoints(folderPath);
}

void Profiler::dumpToFolderTimingLogs(const std::string& folderPath)
{
	static const string prefix = "timing_";
	ofstream file;

	for (auto iter = timingLogs.begin(); iter != timingLogs.end(); iter++) {
		const string& token = iter->first;
		const vector<int>& startFrames = iter->second.start_frames;
		const vector<int>& endFrames = iter->second.end_frames;
		const vector<double>& elapsed_time = iter->second.elapsed_time;
		string filePath = folderPath + "/" + prefix + token + ".txt";
		file.open(filePath);
		assert(file.is_open());
		file << "# " << token << endl;
		file << "# start_frame end_frame t" << endl;
		for (int i = 0; i < elapsed_time.size(); i++) {
			file << startFrames[i] << " " << endFrames[i] << " " << elapsed_time[i] << endl;
		}
		file.close();
	}

	if (!isTimingStatsGenerated) {
		generateTimingStats();
	}

	file.open(folderPath + "/" + prefix + "stats.txt");
	assert(file.is_open());
	file << "# stats" << endl;
	file << "# token average_time total_time length" << endl;
	for (int i = 0; i < timingEntries.size(); i++) {
		const TimingStatsEntry& entry = timingEntries[i];
		file << entry.token << " " << entry.averageTime << " " << entry.totalTime << " " << entry.length << endl;
	}
	file.close();
	return;
}

void Profiler::dumpToFolderDataPoints(const std::string& folderPath)
{
	static const string prefix = "data_poinits_";
	ofstream file;

	for (auto iter = dataPointsLogs.begin(); iter != dataPointsLogs.end(); iter++) {
		const string& token = iter->first;
		const vector<double>& X = iter->second.X;
		const vector<double>& Y = iter->second.Y;
		string filePath = folderPath + "/" + prefix + token + ".txt";
		file.open(filePath);
		assert(file.is_open());
		file << "# " << token << endl;
		file << "# x y" << endl;
		for (int i = 0; i < X.size(); i++) {
			file << X[i] << " " << Y[i] << endl;
		}
		file.close();
	}
	return;
}