#pragma once

#include <map>
#include <memory>

class PrimitiveParam{
	
public:
	PrimitiveParam(){}

	virtual PrimitiveParam* clone() = 0;

	virtual bool stepForward(int i, double step) = 0;
	virtual int numParams() = 0;
	virtual void print() = 0;
	virtual bool setParams( std::vector< double >& newParams ) = 0;
};


// With deep copying
class PrimitiveParamMap{

	typedef std::map< unsigned int, std::shared_ptr<PrimitiveParam> >::iterator Iterator;

public:

	PrimitiveParamMap& operator = (PrimitiveParamMap& from)
	{
		this->params.clear();

		for(Iterator it= from.begin(); it != from.end(); it++)
			this->params[it->first] = std::shared_ptr<PrimitiveParam>(it->second->clone());	

		return *this;
	}

	std::shared_ptr<PrimitiveParam>& operator[](unsigned int i)
	{
		return params[i];
	}

	void print()
	{
		for(Iterator it= params.begin(); it != params.end(); it++){
			it->second->print();
			printf("\n");
		}
	}

	Iterator begin(){return params.begin();}
	Iterator end(){return params.end();}

	std::map< unsigned int, std::shared_ptr<PrimitiveParam> > params;
};

// Vector with deep copying
class PrimitiveParamVector{

	typedef std::vector< std::shared_ptr<PrimitiveParam> >::iterator Iterator;

public:
	PrimitiveParamVector& operator = (PrimitiveParamVector& from)
	{
		this->params.clear();

		for(Iterator it= from.begin(); it != from.end(); it++)
			this->params.push_back( std::shared_ptr<PrimitiveParam>((*it)->clone()) );	

		return *this;
	}

	std::shared_ptr<PrimitiveParam>& operator[](unsigned int i)
	{
		return params[i];
	}

	Iterator begin(){return params.begin();}
	Iterator end(){return params.end();}

	std::vector< std::shared_ptr<PrimitiveParam> > params;
};