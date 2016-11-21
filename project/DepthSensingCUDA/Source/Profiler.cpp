#include "stdafx.h"
#include "Profiler.h"
#include <iostream>
#include <algorithm>

#define EXIT(STATUS)  {system("pause"); exit(STATUS);}

using namespace std;

void Profiler::startTiming(const string& token)
{
	if (timingLogs.find(token) == timingLogs.end()) {
		timingLogs[token] = TimingLog(token);
	}

	TimingLog& log = timingLogs[token];
	if (log.isTiming) {
		cout << "[startTiming] Error: the profiler is still timing another session for [" << token.c_str() << "]." << endl;
		EXIT(-1);
	}

	log.isTiming = true;
	log.start = clock();
}

void Profiler::stopTiming(const string& token)
{
	while (true)
	{
		if (timingLogs.find(token) == timingLogs.end()) break;
		TimingLog& log = timingLogs[token];
		if (!log.isTiming) break;

		log.end = clock();
		log.elapsed_time.push_back(((double)log.end - log.start) / CLOCKS_PER_SEC * 1000.);
		log.start = log.end = 0;
		log.isTiming = false;
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

bool Profiler::StatsEntryCompare(Profiler::StatsEntry& lhs, Profiler::StatsEntry& rhs) {
	return lhs.totalTime > rhs.totalTime;
}

void Profiler::generateTimingStats()
{
	entries.clear();
	for (auto it = timingLogs.begin(); it != timingLogs.end(); it++)
	{
		StatsEntry entry(it->first);
		vector<double> &elapsed_time = it->second.elapsed_time;
		for (int i = 0; i < elapsed_time.size(); i++)
		{
			entry.totalTime += elapsed_time[i];
		}
		entry.averageTime = ((double)entry.totalTime) / elapsed_time.size();
		entries.push_back(entry);
	}
	std::sort(entries.begin(), entries.end(), StatsEntryCompare);
}

void Profiler::printTimingStats()
{
	printf("=================Time Stats=================\n");
	printf("id\tbucket name\ttotal time\taverage time\n");
	for (int i = 0; i < entries.size(); i++)
		printf("[%d]\t%s\t%.2lfms\t%.4lfms\n", i + 1, entries[i].token.c_str(), entries[i].totalTime, entries[i].averageTime);
	printf("===================END======================\n");
	return;
}

void Profiler::clear()
{
	entries.clear();
	timingLogs.clear();
}