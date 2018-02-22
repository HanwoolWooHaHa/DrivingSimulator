#pragma once

#include <QtOpenGL/QGLWidget>

class CDatabase;

class CBirdView : public QGLWidget
{
	Q_OBJECT

public:
	explicit CBirdView( QWidget* parent = 0 );
	~CBirdView() {}

	void Initialize( void );
	void Update( int tick );
    void DrawTrajectory( int nNoTrajectory, bool bFlag );

private:
	void drawVehiclesInDS(void);
	void drawVehicle( int type, int index );
    void drawGround( void );
    void drawLine( void );
    void drawHighway( void );
    void drawGroundTruth( void );
    void drawPredictedTrajectory(void);
    void drawPrecedingTrajectory(void);
    void drawLeadTrajectory(void);
    void drawText( double x, double y, double z );

    void renderVehicle(double dPosX, double dPosY, double dAttitude, double dLength, double dWidth, const QColor& color);

	void cylinder(double dPosX, double dPosY, double dPosZ, float radius, float width, int sides);
	
	CDatabase* m_pDatabase;

	int m_nTick;
	float m_fZoom;
	QPoint m_lastPt;

    bool m_bDrawFlagGround;
    bool m_bDrawFlagTarget;
    bool m_bDrawFlagPreceding;
    bool m_bDrawFlagLead;

    double m_dViewPointX;
    double m_dViewPointY;

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);
	void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
};
