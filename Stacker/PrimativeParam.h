#pragma once

#include <map>

class PrimitiveParam{
	
public:
	PrimitiveParam(){}

	virtual PrimitiveParam* clone() = 0;

	virtual bool stepForward(int i, double step) = 0;
	virtual int numParams() = 0;
	virtual void print() = 0;
};

// CRTP
template<typename T, typename Derive> 
class CloneImpl : public Derive {
public:
	virtual Derive* clone() {return new T(static_cast<const T&>(*this));}
};

// With deep copying
class PrimitiveParamMap{
public:
	PrimitiveParamMap clone(){
		PrimitiveParamMap result;
		for(std::map< unsigned int, PrimitiveParam * >::iterator it = params.begin(); it != params.end(); it++)
			result[it->first] = it->second->clone();
		return result;
	}

	PrimitiveParam*& operator[](unsigned int i){
		return params[i];
	}

	std::map< unsigned int, PrimitiveParam * > params;
};
