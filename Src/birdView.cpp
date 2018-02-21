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

    this->setFixedSize( 1200, 450 );
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
    //qglClearColor(QColor(160, 216, 239));

    qglClearColor(Qt::black);
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
        dGlobalPosX += 40.0; // to change the center point of view

		gluLookAt(dGlobalPosX - m_fZoom, dGlobalPosY + m_fZoom, m_fZoom, dGlobalPosX, dGlobalPosY, 0.0, -0.4962, 0.4962, 0.7017);

        //gluLookAt(dGlobalPosX, dGlobalPosY, m_fZoom, dGlobalPosX, dGlobalPosY, 0.0, -1.0, 0.0, 0.0);
        //gluLookAt(dGlobalPosX + m_fZoom, dGlobalPosY - m_fZoom, m_fZoom, dGlobalPosX, dGlobalPosY, 0.0, 0.4962, -0.4962, 0.7017);
	}

	glMatrixMode(GL_MODELVIEW);

    // 3. draw the highway
    drawGround();
    drawHighway();
    drawLine();

    //drawText(dGlobalPosX, dGlobalPosY - 2.0, 1);

	// 4. draw vehicles
	drawVehiclesInDS();

#if defined(TRAJECTORY_PREDICTION)
    drawPredictedTrajectory();
    drawGroundTruth();
#endif
	
    glFlush();
}
/*********************************************************************/
void CBirdView::drawText(double x, double y, double z)
{
    QString txt;

	int nCurrentTrial = m_pDatabase->GetCurrentTrial();
	int nNumTrial = m_pDatabase->GetNumTrial();
	int nMeasurementTime = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, 1);

	if (nCurrentTrial < 0 || nCurrentTrial > nNumTrial)
		nCurrentTrial = 0;

    //txt = "Trial = " + QString::number(nCurrentTrial) + ", Class = " + QString::number(nClass) + ", Time = " + QString::number(nMeasurementTime) + " s";
    txt = "Trial = " + QString::number(nCurrentTrial) + ", Time = " + QString::number(nMeasurementTime) + " s";

    glLoadIdentity();

    qglColor(Qt::black);
    renderText(x + 1.0, y, z, txt, QFont("Arial", 12, QFont::Bold, false) );
}

void CBirdView::drawGround( void )
{
    // 車線点群をすべて描く
    glLoadIdentity();
    qglColor(Qt::darkGray);

    glEnable(GL_LINE);
    //glLineStipple(1, 0xFCFC);
    glLineWidth(0.3);

    glBegin(GL_LINES);


    //! Only draw the centerline
    for(int i=-8; i<8; i++)
    {
        double dPosX1 = -300.0;
        double dPosX2 = 3000.0;

        double dPosY1 = DS_CENTERLINE + 50 * i;
        double dPosY2 = DS_CENTERLINE + 50 * i;

        GLdouble line[2][3] = { { dPosX1, dPosY1, 0.0 }, { dPosX2, dPosY2, 0.0 } };

        glVertex3dv(line[0]);
        glVertex3dv(line[1]);
    }

    for(int i=0; i<60; i++)
    {
        double dPosX1 = -300.0 + 50 * i;
        double dPosX2 = -300.0 + 50 * i;

        double dPosY1 = DS_CENTERLINE - 400;
        double dPosY2 = DS_CENTERLINE + 400;

        GLdouble line[2][3] = { { dPosX1, dPosY1, 0.0 }, { dPosX2, dPosY2, 0.0 } };

        glVertex3dv(line[0]);
        glVertex3dv(line[1]);
    }


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
/*
    double dDelta = 0.0;

    while(dDelta < 3000.0)
    {
        glBegin(GL_POLYGON);

		glVertex3f(dDelta, -2240.0, 0.0);
		glVertex3f(dDelta + 1.0, -2240.0, 0.0);
		glVertex3f(dDelta + 1.0, -2241.0, 0.0);
		glVertex3f(dDelta, -2241.0, 0.0);

        dDelta += 30.0;

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

        dDelta += 30.0;

		glEnd();
	}
*/
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

    /*
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
    */

    //! Only draw the centerline
    double dPosX1 = -300.0;
    double dPosX2 = 3000.0;

    double dPosY1 = -2248.4;
    double dPosY2 = -2248.4;

    GLdouble line[2][3] = { { dPosX1, dPosY1, 0.01 }, { dPosX2, dPosY2, 0.01 } };

    glVertex3dv(line[0]);
    glVertex3dv(line[1]);

    glEnd();
}

void CBirdView::drawGroundTruth( void )
{
    double dLength = 4.54;
    double dWidth = 1.84;

    dLength *= 0.5;
    dWidth *= 0.5;

    glLoadIdentity();
    qglColor(Qt::red);

    glPointSize(5);
    glBegin(GL_LINES);

    int nNumIndexGroundTruth = (int)(TRAJECTORY_PREDICTION_TIME / DS_DELTA_T); // prediction time X 120 Hz;
    int nIndex = (int)(DS_TRJ_DRW_DELTA / DS_DELTA_T);

    for (int i = 0; i <= nNumIndexGroundTruth; i+=nIndex)
    {
        int nCurrentTrial = m_pDatabase->GetCurrentTrial();
        double dPosX = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick+i, DS_OWN_X);
        double dPosY = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick+i, DS_OWN_Y);

        GLdouble line[4][3] = { { dPosX - dLength, dPosY - dWidth, 0.012 }, { dPosX + dLength, dPosY - dWidth, 0.012 }, { dPosX + dLength, dPosY + dWidth, 0.012 }, { dPosX - dLength, dPosY + dWidth, 0.012 } };

        glVertex3dv(line[0]);
        glVertex3dv(line[1]);

        glVertex3dv(line[1]);
        glVertex3dv(line[2]);

        glVertex3dv(line[2]);
        glVertex3dv(line[3]);

        glVertex3dv(line[3]);
        glVertex3dv(line[0]);
    }
/*
    for (int i = 0; i < nNumIndexGroundTruth; i+=nIndex)
    {
        int nCurrentTrial = m_pDatabase->GetCurrentTrial();

        double dPosX1 = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick+i, DS_OWN_X);
        double dPosY1 = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick+i, DS_OWN_Y);
        double dPosX2 = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick+i+nIndex, DS_OWN_X);
        double dPosY2 = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick+i+nIndex, DS_OWN_Y);

        if(dPosX2 < dPosX1)
            break;

        GLdouble line[2][3] = { { dPosX1, dPosY1, 0.01 }, { dPosX2, dPosY2, 0.01 } };

        glVertex3dv(line[0]);
        glVertex3dv(line[1]);
    }
*/

    glEnd();
}

void CBirdView::drawPredictedTrajectory(void)
{
    double dLength = 4.54;
    double dWidth = 1.84;

    dLength *= 0.5;
    dWidth *= 0.5;

    glLoadIdentity();
    qglColor(Qt::green);

    glPointSize(5);
    glBegin(GL_LINES);

    int nIndexPredictedTrajectory = (int)(TRAJECTORY_PREDICTION_TIME / DS_TRJ_PRD_DELTA); // prediction time X 10 Hz;
    int nIndex = (int)(DS_TRJ_DRW_DELTA / DS_TRJ_PRD_DELTA);

    for (int i = 0; i <= nIndexPredictedTrajectory; i+=nIndex)
    {
        double dPosX = 0.0;
        double dPosY = 0.0;

        CDatabase::GetInstance()->GetPredictedTrajectory(i, &dPosX, &dPosY);

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

    for (int i = 0; i <= nIndexPredictedTrajectory; i+=nIndex)
    {
        double dPosX1 = 0.0;
        double dPosY1 = 0.0;
        double dPosX2 = 0.0;
        double dPosY2 = 0.0;

        if(i==0)
        {
            int nCurrentTrial = m_pDatabase->GetCurrentTrial();
            dPosX1 = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, DS_OWN_X);
            dPosY1 = m_pDatabase->GetData(DS, nCurrentTrial, m_nTick, DS_OWN_Y);
        }
        else
            CDatabase::GetInstance()->GetPredictedTrajectory(i-nIndex, &dPosX1, &dPosY1);

        CDatabase::GetInstance()->GetPredictedTrajectory(i, &dPosX2, &dPosY2);

        GLdouble line[2][3] = { { dPosX1, dPosY1, 0.01 }, { dPosX2, dPosY2, 0.01 } };

        glVertex3dv(line[0]);
        glVertex3dv(line[1]);
    }

    glEnd();
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
            color = Qt::blue;
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
        color = Qt::yellow;
        break;
	}

    dGlobalX *= FEET_TO_METER;
    dGlobalY *= FEET_TO_METER;
    dLength *= FEET_TO_METER;
    dWidth *= FEET_TO_METER;

    //! 車を指定の位置に描く
    if( nVehicleType == TARGET )
        renderVehicle(dGlobalX, dGlobalY, dAttitude, dLength, dWidth, color);

    bool bShowOthers = m_pDatabase->GetFlagShowOthers();
    if( bShowOthers )
    {
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
