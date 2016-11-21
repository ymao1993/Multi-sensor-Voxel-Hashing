#ifndef PROFILER_H
#define PROFILER_H

#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <time.h>

class Profiler
{
public:
	struct TimingLog
	{
		std::string token;
		std::vector<double> elapsed_time;
		clock_t start, end;
		bool isTiming;

		TimingLog(std::string token) :
			token(token),
			isTiming(0),
			start(0),
			end(0)
		{}

		TimingLog():
			token("unknown"),
			isTiming(0),
			start(0),
			end(0)
		{}
	};

	struct StatsEntry
	{
		std::string token;
		double totalTime;
		double averageTime;
		StatsEntry(const std::string& token) :token(token), totalTime(0), averageTime(0) {}
	};

public:
	void startTiming(const std::string& token);
	void stopTiming(const std::string& token);
	void printTimingLog(const std::string& token);
	void generateTimingStats();
	void printTimingStats();
	void clear();

private:
	std::unordered_map<std::string, TimingLog> timingLogs;
	std::vector<StatsEntry> entries;
	static bool StatsEntryCompare(StatsEntry& lhs, StatsEntry& rhs);
};

#endif