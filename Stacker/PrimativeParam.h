#pragma once

#include <map>

class PrimitiveParam{
	
public:
	PrimitiveParam(){}

	virtual PrimitiveParam* clone() = 0;

	virtual bool stepForward(int i, double step) = 0;
	virtual int numParams() = 0;
	virtual void print() = 0;
	virtual bool setParams( std::vector< double >& newParams ) = 0;
	virtual bool forceParam( int i, double val ) = 0;
};

// With deep copying
class PrimitiveParamMap{

	typedef std::map< unsigned int, PrimitiveParam* >::iterator Iterator;

public:
	~PrimitiveParamMap()
	{
		this->clear();
	}

	PrimitiveParamMap& operator = (PrimitiveParamMap& from)
	{
		this->clear();

		for(Iterator it= from.begin(); it != from.end(); it++)
			this->params[it->first] = it->second->clone();	

		return *this;
	}

	PrimitiveParam*& operator[](unsigned int i)
	{
		return params[i];
	}

	void clear()
	{
		for(Iterator it= params.begin(); it != params.end(); it++)
			delete it->second;

		params.clear();
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

	std::map< unsigned int, PrimitiveParam* > params;
};

// Vector with deep copying
class PrimitiveParamVector{

	typedef std::vector< PrimitiveParam* >::iterator Iterator;

public:
	~PrimitiveParamVector()
	{
		this->clear();
	}

	PrimitiveParamVector& operator = (PrimitiveParamVector& from)
	{
		this->clear();

		for(Iterator it= from.begin(); it != from.end(); it++)
			this->params.push_back( (*it)->clone() );	

		return *this;
	}

	PrimitiveParam*& operator[](unsigned int i)
	{
		return params[i];
	}

	void clear()
	{
		for(Iterator it= params.begin(); it != params.end(); it++)
			delete *it;

		params.clear();
	}

	Iterator begin(){return params.begin();}
	Iterator end(){return params.end();}
	void push_back(PrimitiveParam* param){params.push_back(param);}


	std::vector< PrimitiveParam* > params;
};