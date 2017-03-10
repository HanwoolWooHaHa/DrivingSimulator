#include "../Include/birdView.h"
#include "../Include/database.h"

#include <GL/glu.h>
#include <QWheelEvent>
#include <qmath.h>
#include <QDebug>

/*********************************************************************/
CBirdView::CBirdView(QWidget *parent) : QGLWidget(parent)
{
	m_pDatabase = CDatabase::GetInstance();

	this->setFixedSize( 800, 300 );
	m_nTick = 0;
}
/*********************************************************************/
/* Public functions */
void CBirdView::Update( int tick )
{
	m_nTick = tick;
	update();
}

void CBirdView::Initialize( void )
{
	m_fZoom = 60.0;
	m_lastPt.setX( 0.0);
    m_lastPt.setY( 0.0);
}
/*********************************************************************/
/* Private member functions */
void CBirdView::initializeGL()
{
    //qglClearColor( QColor(64, 32, 64) );
	qglClearColor(QColor(160, 216, 239));
    glEnable( GL_DEPTH_TEST );
}

void CBirdView::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    glLoadIdentity();

    //glOrtho(-1 , 1 , -1 , 1 , 2 , 4);
    glOrtho(-1 , 1 , -1 , 1 , -1, 1);
}

void CBirdView::wheelEvent(QWheelEvent *event)
{
	float delta = (float)event->delta();

    m_fZoom += (delta/120.0);

    updateGL();
}

#if 0
void CBirdView::mousePressEvent(QMouseEvent *event)
{
	m_lastPt = event->pos();
}

void CBirdView::mouseMoveEvent(QMouseEvent *event)
{
	float dx = (event->x() - m_lastPt.x()) / 10.0f;
	float dy = (event->y() - m_lastPt.y())/ 10.0f;

	if (event->buttons() && Qt::LeftButton) 
    {
		glRotatef(dy*0.1, 1.0f, 0.0f, 0.0f);
		glRotatef(dx*0.1, 0.0f, 1.0f, 0.0f);
    }
}
#endif

void CBirdView::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();
	gluPerspective(30.0, (double)width() / (double)height(), 1.0, 1000.0);

	int nCurrentTrial = m_pDatabase->GetCurrentTrial();
	int nNumTrial = m_pDatabase->GetNumTrial();

	if (nCurrentTrial < 0 || nCurrentTrial > nNumTrial)
		nCurrentTrial = 0;

    double dGlobalPosX = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, DS_OWN_X);
    double dGlobalPosY = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, DS_OWN_Y);
    //double dAttitude = 0; //m_pDatabase->GetData(CDatabase::MEASUREMENT, m_nTick, PACKET_OWN_GLOBAL_ATT);

	// 2. set the view point
	if( m_nTick == 0 )
	{
		gluLookAt( 25.0, 15.0, 20.0, 0.0, 0.0, 0.0,  0.0, 0.0, 1.0);
		glMatrixMode(GL_MODELVIEW);
		glFlush();
		return;
	}
	else
	{
		//! Global座標系における自車の真上から見た視点
        //gluLookAt(dGlobalPosX, dGlobalPosY, m_fZoom, dGlobalPosX, dGlobalPosY, 0.0, -1.0, 0.0, 0.0);

        //gluLookAt(dGlobalPosX + m_fZoom, dGlobalPosY - m_fZoom, m_fZoom, dGlobalPosX, dGlobalPosY, 0.0, 0.4962, -0.4962, 0.7017);

		gluLookAt(dGlobalPosX - m_fZoom, dGlobalPosY + m_fZoom, m_fZoom, dGlobalPosX, dGlobalPosY, 0.0, -0.4962, 0.4962, 0.7017);
	}

	glMatrixMode(GL_MODELVIEW);

    // 3. draw the highway
    drawHighway();
    drawLine();

	drawText(dGlobalPosX, dGlobalPosY - 2.0, 1);

	// 4. draw vehicles
	drawVehiclesInDS();
	
    glFlush();
}
/*********************************************************************/
void CBirdView::drawText(double x, double y, double z)
{
    QString txt;

	int nCurrentTrial = m_pDatabase->GetCurrentTrial();
	int nNumTrial = m_pDatabase->GetNumTrial();
	int nMeasurementTime = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, 1);
    int nClass = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, DS_CLASS);

	if (nCurrentTrial < 0 || nCurrentTrial > nNumTrial)
		nCurrentTrial = 0;

    //txt = "Trial = " + QString::number(nCurrentTrial) + ", Class = " + QString::number(nClass) + ", Time = " + QString::number(nMeasurementTime) + " s";
    txt = "Trial = " + QString::number(nCurrentTrial) + ", Time = " + QString::number(nMeasurementTime) + " s";

    glLoadIdentity();

    qglColor(Qt::black);
    renderText(x + 1.0, y, z, txt, QFont("Arial", 12, QFont::Bold, false) );
}

void CBirdView::drawHighway(double dGlobalOwnX, double dGlobalOwnY, double dAttitude)
{
	glPushMatrix();
	glLoadIdentity();

    glTranslated(dGlobalOwnX, dGlobalOwnY, 0.0);
    glRotatef(dAttitude, 0.0, 0.0, 1.0);

    qglColor(Qt::darkGray);

	// 윗면
	glBegin(GL_POLYGON);

    glVertex3f(-1000.0, -10.0, 0.0);
    glVertex3f(-1000.0, 10.0, 0.0);
    glVertex3f(1000.0, 10.0, 0.0);
    glVertex3f(1000.0, -10.0, 0.0);

	glEnd();
}

void CBirdView::drawHighway( void )
{
    glPushMatrix();
    glLoadIdentity();

    qglColor(Qt::darkGray);

    // 윗면
    glBegin(GL_POLYGON);

    glVertex3f(-300.0, -2244.8, 0.0);
    glVertex3f(-300.0, -2252.0, 0.0);
	glVertex3f(3000.0, -2252.0, 0.0);
	glVertex3f(3000.0, -2244.8, 0.0);

    glEnd();

    double dDelta = 0.0;

    while(dDelta < 3000.0)
    {
        glBegin(GL_POLYGON);

		glVertex3f(dDelta, -2240.0, 0.0);
		glVertex3f(dDelta + 1.0, -2240.0, 0.0);
		glVertex3f(dDelta + 1.0, -2241.0, 0.0);
		glVertex3f(dDelta, -2241.0, 0.0);

        dDelta += 10.0;

        glEnd();
    }

	dDelta = 0.0;

	while (dDelta < 3000.0)
	{
		glBegin(GL_POLYGON);

		glVertex3f(dDelta, -2255.0, 0.0);
		glVertex3f(dDelta + 1.0, -2255.0, 0.0);
		glVertex3f(dDelta + 1.0, -2256.0, 0.0);
		glVertex3f(dDelta, -2256.0, 0.0);

		dDelta += 10.0;

		glEnd();
	}
}

void CBirdView::drawLine( void )
{
	double dLinePosY[3] = { -2244.8, -2248.4, -2252.0 };

	// 車線点群をすべて描く
	glLoadIdentity();
	qglColor(Qt::white);

	glEnable(GL_LINE_STIPPLE);
	glLineStipple(1, 0xFCFC);
    glLineWidth(3.0);

	glBegin(GL_LINES);

    for (int n = 0; n<NUM_LINE; n++)
	{
        double dPosX1 = -300.0;
		double dPosX2 = 3000.0;

		double dPosY1 = dLinePosY[n];
		double dPosY2 = dLinePosY[n];

        GLdouble line[2][3] = { { dPosX1, dPosY1, 0.01 }, { dPosX2, dPosY2, 0.01 } };

        glVertex3dv(line[0]);
        glVertex3dv(line[1]);
	}

    glEnd();
}

void CBirdView::drawTargetVehicleTrajectory( void )
{
    if( m_nTick == 0 )
        return;

    int nDataLength = CDatabase::GetInstance()->GetDataLength();

    glLoadIdentity();
    qglColor(Qt::red);

    glEnable(GL_LINE_STIPPLE);
    glLineStipple(1, 0xFFFF);

    glBegin(GL_LINES);

    for( int t=m_nTick; t<nDataLength; t++ )
    {
        double dPreGlobalPosX = m_pDatabase->GetData(TARGET, 0, t-1, DATA_PACKET_X) * FEET_TO_METER;
        double dPreGlobalPosY = m_pDatabase->GetData(TARGET, 0, t-1, DATA_PACKET_Y) * FEET_TO_METER;
        double dGlobalPosX = m_pDatabase->GetData(TARGET, 0, t, DATA_PACKET_X) * FEET_TO_METER;
        double dGlobalPosY = m_pDatabase->GetData(TARGET, 0, t, DATA_PACKET_Y) * FEET_TO_METER;

        GLdouble line[2][3] = { { dPreGlobalPosX, dPreGlobalPosY, 0.01 }, { dGlobalPosX, dGlobalPosY, 0.01 } };

        glVertex3dv(line[0]);
        glVertex3dv(line[1]);
    }

    glEnd();
}

void CBirdView::drawPredictedTrajectory(void)
{
	glLoadIdentity();
	qglColor(Qt::green);

	glPointSize(5);
	glBegin(GL_POINTS);

	for (int i = 0; i < (int)(TRAJECTORY_PREDICTION_TIME * 10); i++)
	{
		double dPosX = 0.0;
		double dPosY = 0.0;
		double dPrePosX = 0.0;
		double dPrePosY = 0.0;

		m_pDatabase->GetPredictedTrajectory(i, &dPosX, &dPosY);
		m_pDatabase->GetPredictedTrajectory(i-1, &dPrePosX, &dPrePosY);

		glVertex3f(dPosX, dPosY, 0.01);
	}

	glEnd();
}

void CBirdView::drawPredictedPosition(void)
{
	double dLength = m_pDatabase->GetData(TARGET, 0, m_nTick, DATA_PACKET_WIDTH);
	double dWidth = m_pDatabase->GetData(TARGET, 0, m_nTick, DATA_PACKET_LENGTH);

	dLength = 0.5 * dLength * FEET_TO_METER;
	dWidth = 0.5 * dWidth * FEET_TO_METER;

	glLoadIdentity();
	qglColor(Qt::green);

	glPointSize(5);
	glBegin(GL_LINES);

	for (int i = 0; i < (int)(TRAJECTORY_PREDICTION_TIME * 10); i++)
	{
		double dPosX = 0.0;
		double dPosY = 0.0;

		m_pDatabase->GetPredictedTrajectory(i, &dPosX, &dPosY);
		
		GLdouble line[4][3] = { { dPosX - dLength, dPosY - dWidth, 0.01 }, { dPosX + dLength, dPosY - dWidth, 0.01 }, { dPosX + dLength, dPosY + dWidth, 0.01 }, { dPosX - dLength, dPosY + dWidth, 0.01 } };
		
		glVertex3dv(line[0]);
		glVertex3dv(line[1]);

		glVertex3dv(line[1]);
		glVertex3dv(line[2]);

		glVertex3dv(line[2]);
		glVertex3dv(line[3]);

		glVertex3dv(line[3]);
		glVertex3dv(line[0]);
	}

	glEnd();

	if (CDatabase::GetInstance()->GetCollisionFlag())
	{
		qglColor(Qt::black);

		glPointSize(5);
		glBegin(GL_LINES);

		for (int i = 0; i < (int)(TRAJECTORY_PREDICTION_TIME * 10); i++)
		{
			double dPosX = 0.0;
			double dPosY = 0.0;

			m_pDatabase->GetRePredictedTrajectory(i, &dPosX, &dPosY);

			GLdouble line[4][3] = { { dPosX - dLength, dPosY - dWidth, 0.01 }, { dPosX + dLength, dPosY - dWidth, 0.01 }, { dPosX + dLength, dPosY + dWidth, 0.01 }, { dPosX - dLength, dPosY + dWidth, 0.01 } };

			glVertex3dv(line[0]);
			glVertex3dv(line[1]);

			glVertex3dv(line[1]);
			glVertex3dv(line[2]);

			glVertex3dv(line[2]);
			glVertex3dv(line[3]);

			glVertex3dv(line[3]);
			glVertex3dv(line[0]);
		}

		glEnd();
	}
}

void CBirdView::drawLineMarkings(void)
{
#if 0
	// 車線点群をすべて描く
	glLoadIdentity();
	qglColor(Qt::white);

	glPointSize(6);
	glEnable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);

	int nMax = m_pDatabase->GetNumMarks();

	for (int n = 0; n<NUM_LANE; n++)
	{
		for (int i = 0; i<nMax; i++)
		{
			double dPosX = m_pDatabase->GetData(CDatabase::LANE, n, i, 0);
			double dPosY = m_pDatabase->GetData(CDatabase::LANE, n, i, 1);

			glVertex3f(dPosX, dPosY, 0.1);
		}
	}
	glEnd();
#endif
}

void CBirdView::drawVehiclesInDS(void)
{
	double dGlobalX = 0.0;
	double dGlobalY = 0.0;
	double dAttitude = 0.0;
	double dLength = 4.54;
	double dWidth = 1.84;
	QColor color;

	int nCurrentTrial = m_pDatabase->GetCurrentTrial();

	for (int n = 0; n < 3; n++)
	{
		if (n == 0)
		{
            dGlobalX = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, DS_OWN_X);
            dGlobalY = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, DS_OWN_Y);
			color = Qt::red;
		}
		else if (n==1)
		{
            dGlobalX = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, DS_PRECED_X);
            dGlobalY = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, DS_PRECED_Y);
			color = Qt::yellow;
		}
		else
		{
            dGlobalX = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, DS_LEAD_X);
            dGlobalY = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, DS_LEAD_Y);
			color = Qt::yellow;
		}

		renderVehicle(dGlobalX, dGlobalY, dAttitude, dLength, dWidth, color);
	}
	
}

void CBirdView::drawVehicle( int nVehicleType, int nVehicleIndex )
{
	double dGlobalX = 0.0;
	double dGlobalY = 0.0;
	double dAttitude = 0.0;
    double dLength = 0.0;
    double dWidth = 0.0;
	QColor color;
    int nVehicleNo = 0;

    switch( nVehicleType )
	{
    case TARGET:
        dGlobalX = m_pDatabase->GetData(TARGET, nVehicleIndex, m_nTick, DATA_PACKET_X);
        dGlobalY = m_pDatabase->GetData(TARGET, nVehicleIndex, m_nTick, DATA_PACKET_Y);
        dAttitude = 90.0;
        dLength = m_pDatabase->GetData(TARGET, nVehicleIndex, m_nTick, DATA_PACKET_LENGTH);
        dWidth = m_pDatabase->GetData(TARGET, nVehicleIndex, m_nTick, DATA_PACKET_WIDTH);
        color = Qt::red;
		break;

    case ADJACENT:
        dGlobalX = m_pDatabase->GetData(ADJACENT, nVehicleIndex, m_nTick, ADJ_DATA_PACKET_X);
        dGlobalY = m_pDatabase->GetData(ADJACENT, nVehicleIndex, m_nTick, ADJ_DATA_PACKET_Y);
        dLength = m_pDatabase->GetDataInfo(ADJACENT, nVehicleIndex, 2);
        dWidth = m_pDatabase->GetDataInfo(ADJACENT, nVehicleIndex, 3);
        nVehicleNo = m_pDatabase->GetDataInfo(ADJACENT, nVehicleIndex, 0);
        dAttitude = 90.0;
        color = Qt::yellow;
        break;
	}

    dGlobalX *= FEET_TO_METER;
    dGlobalY *= FEET_TO_METER;
    dLength *= FEET_TO_METER;
    dWidth *= FEET_TO_METER;

    int nLeadVehicleNo = m_pDatabase->GetAdjacentVehicleData( LEAD_VEHICLE_NO );
    int nRearVehicleNo = m_pDatabase->GetAdjacentVehicleData( REAR_VEHICLE_NO );
    int nPrecedingVehicleNo = m_pDatabase->GetAdjacentVehicleData( PRECEDING_VEHICLE_NO );
    int nFollowingVehicleNo = m_pDatabase->GetAdjacentVehicleData( FOLLOWING_VEHICLE_NO );


    if( (nVehicleType == ADJACENT) && (nVehicleNo == nLeadVehicleNo || nVehicleNo == nRearVehicleNo || nVehicleNo == nPrecedingVehicleNo || nVehicleNo == nFollowingVehicleNo) )
        color = Qt::blue;

    //! 車を指定の位置に描く
    if( nVehicleType == TARGET )
        renderVehicle(dGlobalX, dGlobalY, dAttitude, dLength, dWidth, color);

    bool bShowOthers = m_pDatabase->GetFlagShowOthers();
    if( bShowOthers )
    {
        renderVehicle(dGlobalX, dGlobalY, dAttitude, dLength, dWidth, color);
    }
    else
    {
        if( (nVehicleType == ADJACENT) && (nVehicleNo == nLeadVehicleNo || nVehicleNo == nRearVehicleNo || nVehicleNo == nPrecedingVehicleNo || nVehicleNo == nFollowingVehicleNo) )
            renderVehicle(dGlobalX, dGlobalY, dAttitude, dLength, dWidth, color);
    }
}

//円柱
void CBirdView::cylinder(double dPosX, double dPosY, double dPosZ, float radius, float width, int sides)
{
	glColor3ub(15, 15, 15);
	glTranslated(dPosX, dPosY, dPosZ);

	double pi = 3.1415;
	//上面
	glNormal3d(0.0, 1.0, 0.0);
	glBegin(GL_POLYGON);
	for (double i = 0; i < sides; i++) {
		double t = pi * 2 / sides * (double)i;
		glVertex3d(radius * cos(t), width / 2, radius * sin(t));
	}
	glEnd();
	//側面
	qglColor(Qt::black);
	glBegin(GL_QUAD_STRIP);
	for (double i = 0; i <= sides; i = i + 1){
		double t = i * 2 * pi / sides;
		glNormal3f((GLfloat)cos(t), 0.0, (GLfloat)sin(t));
		glVertex3f((GLfloat)(radius*cos(t)), -width / 2, (GLfloat)(radius*sin(t)));
		glVertex3f((GLfloat)(radius*cos(t)), width / 2, (GLfloat)(radius*sin(t)));
	}
	glEnd();
	//下面
	glColor3ub(15, 15, 15);
	glNormal3d(0.0, -1.0, 0.0);
	glBegin(GL_POLYGON);
	for (double i = sides; i >= 0; --i) {
		double t = pi * 2 / sides * (double)i;
		glVertex3d(radius * cos(t), -width / 2, radius * sin(t));
	}
	glEnd();
}

void CBirdView::renderVehicle(double dPosX, double dPosY, double dAttitude, double dLength, double dWidth, const QColor& color)
{
	glPushMatrix();

	//! タイヤを描く
	double dTirePos[4][3] = { { 0.4 * dLength, 0.5 * dWidth, 0.362 }, { 0.4 * dLength, -0.5 * dWidth, 0.362 }, { -0.4 * dLength, 0.5 * dWidth, 0.362 }, { -0.4 * dLength, -0.5 * dWidth, 0.362 } };
	for (int i = 0; i < 4; i++)
	{
		glLoadIdentity();
		glTranslated(dPosX, dPosY, 0.0);
		glRotatef(dAttitude, 0.0, 0.0, 1.0);

		cylinder(dTirePos[i][0], dTirePos[i][1], dTirePos[i][2], 0.362f, 0.2f, 20);
	}

	glLoadIdentity();
	glTranslated(dPosX, dPosY, 0.0);
	glRotatef(dAttitude, 0.0, 0.0, 1.0);

	
	if (dLength >= 5.0) // like a bus
	{
		//! 高さ指定
		double h1 = 1.705;
		double h0 = 0.362;

		//! sides
		double dx0 = 0.5 * dLength;
		double dx1 = -0.5 * dLength;
		double dy0 = 0.5 * dWidth;
		double dy1 = -0.5 * dWidth;

		// ----------------------- 옆면 4개
		qglColor(color);
		glBegin(GL_QUAD_STRIP);
		glVertex3f(dx0, dy0, h0);
		glVertex3f(dx0, dy0, h1);

		glVertex3f(dx1, dy0, h0);
		glVertex3f(dx1, dy0, h1);

		glVertex3f(dx1, dy1, h0);
		glVertex3f(dx1, dy1, h1);

		glVertex3f(dx0, dy1, h0);
		glVertex3f(dx0, dy1, h1);


		glEnd();

		// 윗면
		glBegin(GL_POLYGON);
		glVertex3f(dx0, dy0, h1);
		glVertex3f(dx0, dy1, h1);
		glVertex3f(dx1, dy1, h1);
		glVertex3f(dx1, dy0, h1);
		glEnd();



		// 모서리에 라인을 그려서 강조
		glColor3ub(0, 0, 0);
		//glColor3ub(86, 21, 10);
		glLineStipple(1, 0xFFFF);
		glLineWidth(3.0);

		glBegin(GL_LINES);
		glVertex3f(dx0, dy0, h1);
		glVertex3f(dx1, dy0, h1);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(dx0, dy0, h1);
		glVertex3f(dx0, dy1, h1);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(dx1, dy0, h1);
		glVertex3f(dx1, dy1, h1);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(dx0, dy1, h1);
		glVertex3f(dx1, dy1, h1);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(dx0, dy0, h0);
		glVertex3f(dx0, dy0, h1);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(dx1, dy0, h0);
		glVertex3f(dx1, dy0, h1);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(dx0, dy1, h0);
		glVertex3f(dx0, dy1, h1);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(dx1, dy1, h0);
		glVertex3f(dx1, dy1, h1);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(dx0, dy0, h0);
		glVertex3f(dx1, dy0, h0);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(dx0, dy0, h0);
		glVertex3f(dx0, dy1, h0);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(dx1, dy0, h0);
		glVertex3f(dx1, dy1, h0);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(dx0, dy1, h0);
		glVertex3f(dx1, dy1, h0);
		glEnd();
	}
	else
	{
		//! 高さ指定
		double h0 = 0.362;
		double h1 = 1.0;
		double h2 = 1.2;
		double h3 = 1.705;

		double dx0 = -0.5 * dLength;
		double dx1 = 0.1 * dLength;
		double dx2 = 0.3 * dLength;
		double dx3 = 0.5 * dLength;

		double dy0 = -0.5 * dWidth;
		double dy1 = 0.5 * dWidth;

		qglColor(color);
		glBegin(GL_QUAD_STRIP);

		// Sides
		glVertex3f(dx0, dy1, h3);
		glVertex3f(dx0, dy1, h0);

		glVertex3f(dx1, dy1, h3);
		glVertex3f(dx1, dy1, h0);

		glVertex3f(dx2, dy1, h2);
		glVertex3f(dx2, dy1, h0);

		glVertex3f(dx3, dy1, h1);
		glVertex3f(dx3, dy1, h0);

		glVertex3f(dx3, dy0, h1);
		glVertex3f(dx3, dy0, h0);

		glVertex3f(dx2, dy0, h2);
		glVertex3f(dx2, dy0, h0);

		glVertex3f(dx1, dy0, h3);
		glVertex3f(dx1, dy0, h0);

		glVertex3f(dx0, dy0, h3);
		glVertex3f(dx0, dy0, h0);

		glVertex3f(dx0, dy1, h3);
		glVertex3f(dx0, dy1, h0);
		
		glEnd();

		// Upper
		glBegin(GL_POLYGON);
		glVertex3f(dx0, dy1, h3);
		glVertex3f(dx1, dy1, h3);
		glVertex3f(dx1, dy0, h3);
		glVertex3f(dx0, dy0, h3);
		glEnd();

		// net
		glBegin(GL_POLYGON);
		glVertex3f(dx2, dy1, h2);
		glVertex3f(dx3, dy1, h1);
		glVertex3f(dx3, dy0, h1);
		glVertex3f(dx2, dy0, h2);
		glEnd();

		// window
		glColor3ub(25, 25, 25);
		glBegin(GL_POLYGON);
		glVertex3f(dx1, dy1, h3);
		glVertex3f(dx2, dy1, h2);
		glVertex3f(dx2, dy0, h2);
		glVertex3f(dx1, dy0, h3);
		glEnd();

		// Lines
		glColor3ub(0, 0, 0);
		//glColor3ub(86, 21, 10);
		glLineStipple(1, 0xFFFF);
		glLineWidth(3.0);
		glBegin(GL_LINES);

		glVertex3f(dx0, dy1, h3);
		glVertex3f(dx0, dy1, h0);

		glVertex3f(dx3, dy1, h1);
		glVertex3f(dx3, dy1, h0);

		glVertex3f(dx0, dy0, h3);
		glVertex3f(dx0, dy0, h0);

		glVertex3f(dx3, dy0, h1);
		glVertex3f(dx3, dy0, h0);

		glVertex3f(dx0, dy0, h0);
		glVertex3f(dx3, dy0, h0);

		glVertex3f(dx0, dy0, h3);
		glVertex3f(dx1, dy0, h3);

		glVertex3f(dx1, dy0, h3);
		glVertex3f(dx2, dy0, h2);

		glVertex3f(dx2, dy0, h2);
		glVertex3f(dx3, dy0, h1);

		glVertex3f(dx0, dy1, h0);
		glVertex3f(dx3, dy1, h0);

		glVertex3f(dx0, dy1, h3);
		glVertex3f(dx1, dy1, h3);

		glVertex3f(dx1, dy1, h3);
		glVertex3f(dx2, dy1, h2);

		glVertex3f(dx2, dy1, h2);
		glVertex3f(dx3, dy1, h1);

		glVertex3f(dx0, dy0, h0);
		glVertex3f(dx0, dy1, h0);

		glVertex3f(dx0, dy0, h3);
		glVertex3f(dx0, dy1, h3);

		glVertex3f(dx1, dy0, h3);
		glVertex3f(dx1, dy1, h3);

		glVertex3f(dx3, dy0, h1);
		glVertex3f(dx3, dy1, h1);

		glVertex3f(dx3, dy0, h0);
		glVertex3f(dx3, dy1, h0);

		glEnd();
	}
}
