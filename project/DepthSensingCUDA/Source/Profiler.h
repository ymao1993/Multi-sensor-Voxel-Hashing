#ifndef PROFILER_H
#define PROFILER_H

#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <time.h>

class Profiler
{
// TIMING
// implemented with QPC.
// http://stackoverflow.com/questions/1739259/how-to-use-queryperformancecounter
// https://msdn.microsoft.com/en-us/library/windows/desktop/dn553408(v=vs.85).aspx
	struct TimingLog
	{
		std::string token;
		std::vector<double> elapsed_time;	//ms
		std::vector<int> start_frames;
		std::vector<int> end_frames;
		__int64 start, end;
		bool isTiming;

		TimingLog(std::string token) :
			token(token),
			isTiming(0),
			start(0),
			end(0)
		{}

		TimingLog() :
			token("unknown"),
			isTiming(0),
			start(0),
			end(0)
		{}
	};

	struct TimingStatsEntry
	{
		std::string token;
		double totalTime;
		double averageTime;
		TimingStatsEntry(const std::string& token) :token(token), totalTime(0), averageTime(0) {}
	};
public:
	void startTiming(const std::string& token, int startFrame=0);
	void stopTiming(const std::string& token, int endFrame=0);
	void printTimingLog(const std::string& token);
	void generateTimingStats();
	void printTimingStats();
	void resetTiming();
private:
	std::unordered_map<std::string, TimingLog> timingLogs;
	std::vector<TimingStatsEntry> timingEntries;
	bool isTimingStatsGenerated = false;
	static bool TimingStatsEntryCompare(TimingStatsEntry& lhs, TimingStatsEntry& rhs);

// NUMERICAL DATA POINTS RECORDING
public:
	struct DataPointsLog
	{
		std::string token;
		std::vector<double> Y;
		std::vector<double> X;

		DataPointsLog(std::string token) :
			token(token)
		{}

		DataPointsLog() :
			token("unknown")
		{}
	};
	void recordDataPoints(const std::string& token, float y, float x = 0);
	void resetDataPointsRecording();

private:
	std::unordered_map<std::string, DataPointsLog> dataPointsLogs;

// FILE IO
public:
	void dumpToFolderAll(const std::string& folderPath);
	void dumpToFolderTimingLogs(const std::string& folderPath);
	void dumpToFolderDataPoints(const std::string& folderPath);

// MISC
public:
	void resetAll();
};

static Profiler profile;

#endif