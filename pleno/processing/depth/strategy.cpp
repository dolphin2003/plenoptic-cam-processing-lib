#include "strategy.h"

void save(v::OutputArchive& archive, const DepthEstimationStrategy& strategies)
{
	std::string init 		= to_string(strategies.init);
	std::string pairing 	= to_string(strategies.pairing);
	std::string belief	= to_string(strategies.belief);
	std::string search	= to_string(strategies.search);
	
	bool dtype	= static_cast<bool>(strategies.dtype);
	bool mtype	= static_cast<bool>(strategies.mtype);
	
	archive
		("multithread", strategies.multithread)
		("nbthread", strategies.nbthread)
		("dtype", dtype)
		("mtype", mtype)
		("init", init)
		("randomize", strategies.randomize)
		("pairing", pairing)
		("filter", strategies.filter)
		("belief", belief)
		("search", search)
		("metricspace", strategies.metric)
		("probabilistic", strategies.probabilistic)
		("precision", strategies.precision)
		("vmin", strategies.vmin)
		("vmax", strategies.vmax);
}

void load(v::InputArchive& archive, DepthEstimationStrategy& strategies)
{
	std::string init, pairing, belief, search;
	
	bool dtype = static_cast<bool>(strategies.dtype);
	bool mtype = static_cast<bool>(strategies.mtype);
	
	archive
		("multithread", strategies.multithread)
		("nbthread", strategies.nbthread)
		("virtual", dtype)
		("coarse", mtype)
		("init", init)
		("randomize", strategies.randomize)
		("pairing", pairing)
		("filter", strategies.filter)
		("belief", belief)
		("search", search)
		("metricspace", strategies.metric)
		("probabilistic", strategies.probabilistic)
		("precision", strategies.precision)
		("vmin", strategies.vmin)
		("vmax", strategies.vmax);
		
	from_string(init, strategies.init);
	from_string(pairing, strategies.pairing);
	from_string(belief, strategies.belief);
	from_string(search, strategies.search);
	
	strategies.dtype = DepthMap::DepthType(dtype);
	strategies.mtype = DepthMap::MapType(mtype);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
std::ostream& operator<<(std::ostream& os, const InitStrategy& mode)
{
	os << "Initilisation strategy = ";
	switch(mode)
	{
		case InitStrategy::RANDOM: os << "Random"; break;
		case InitStrategy::REGULAR_GRID: os << "Regular grid pattern"; break;
		case InitStrategy::FROM_LEFT_BORDER: os << "Regular pattern shifted from left"; break;
	}
	return os;
}
std::string to_string(const InitStrategy& mode)
{
	if (mode == InitStrategy::RANDOM) return "RANDOM";
	else if (mode == InitStrategy::REGULAR_GRID) return "REGULAR_GRID";
	else if (mode == InitStrategy::FROM_LEFT_BORDER) return "FROM_LEFT_BORDER";
	else return "RANDOM";
}
void from_string(const std::string& mode, InitStrategy& strat)
{
	if (mode == to_string(InitStrategy::RANDOM)) strat = InitStrategy::RANDOM;
	else if (mode == to_string(InitStrategy::REGULAR_GRID)) strat = InitStrategy::REGULAR_GRID;
	else if (mode == to_string(InitStrategy::FROM_LEFT_BORDER)) strat = InitStrategy::FROM_LEFT_BORDER;
	else strat = InitStrategy::RANDOM;
}
//******************************************************************************
std::ostream& operator<<(std::ostream& os, const BeliefPropagationStrategy& mode)
{
	os << "Belief propagation strategy = ";
	switch(mode)
	{
		case BeliefPropagationStrategy::NONE: os << "No propagation"; break;
		case BeliefPropagationStrategy::FIRST_RING: os << "Propagate to inner ring"; break;
		case BeliefPropagationStrategy::ALL_NEIGHS: os << "Propagate to all neighbors"; break;
	}
	return os;
}
std::string to_string(const BeliefPropagationStrategy& mode)
{
	if (mode == BeliefPropagationStrategy::NONE) return "NONE";
	else if (mode == BeliefPropagationStrategy::FIRST_RING) return "FIRST_RING";
	else if (mode == BeliefPropagationStrategy::ALL_NEIGHS) return "ALL_NEIGHS";
	else return "NONE";
}
void from_string(const std::string& mode, BeliefPropagationStrategy& strat)
{
	if (mode == to_string(BeliefPropagationStrategy::NONE)) strat = BeliefPropagationStrategy::NONE;
	else if 