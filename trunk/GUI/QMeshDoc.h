#pragma once

#include <QObject>
#include <QMap>
#include "QSegMesh.h"


class QMeshDoc : public QObject
{
	Q_OBJECT

public:
	QMeshDoc();
	~QMeshDoc();

	// Objects
	QMap<QString, QSegMesh*> all_objects;
	uint global_id;
	QSegMesh * getObject( QString objectId );

signals:
	void objectImported( QSegMesh* mesh );
	void printMessage( QString message );

public slots:
	void importObject( QString fileName );
	void deleteObject( QString objectId );
};

