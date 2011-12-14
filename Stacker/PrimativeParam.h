#pragma once

#include <QString>
#include <QMap>
#include <map>

class PrimitiveParam{
	
public:
	PrimitiveParam(QString ID){ this->id = ID; }

	virtual PrimitiveParam* clone() = 0;

	virtual bool stepForward(int i, double step) = 0;
	virtual int numParams() = 0;
	virtual void print() = 0;
	virtual bool setParams( std::vector< double >& newParams ) = 0;
	virtual bool forceParam( int i, double val ) = 0;

	QString id;
};

// Map with deep copying
class PrimitiveParamMap{

public:
	~PrimitiveParamMap()
	{
		this->clear();
	}

	PrimitiveParamMap& operator = (PrimitiveParamMap& from)
	{
		this->clear();

		foreach(PrimitiveParam* param, from.params)
			params[param->id] = param->clone();

		return *this;
	}

	PrimitiveParam*& operator[](QString id)
	{
		return params[id];
	}

	void clear()
	{
		foreach(PrimitiveParam* param, params)
			delete param;

		params.clear();
	}

	QMap< QString, PrimitiveParam* > params;
};