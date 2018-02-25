#include "../Include/leastsquare.h"
#include "../Library/dlib/optimization.h"
#include "../Include/database.h"

#include <vector>
#include <iostream>
#include <qmath.h>
#include <QDebug>

using namespace std;
using namespace dlib;

typedef matrix<double,1,1> input_vector;
typedef matrix<double,2,1> parameter_vector;

static double model (const input_vector& input, const parameter_vector& params)
{
    const double p0 = params(0);
    const double p1 = params(1);

    const double i0 = input(0);

    const double temp = -p0 * exp( - i0 * i0 / ( p1 * p1 ) ) * 2.0 * i0 / ( p1 * p1 ) + 0.2;

    return temp*temp;
}

static double residual (const std::pair<input_vector, double>& data, const parameter_vector& params)
{
    return model(data.first, params) - data.second;
}

static parameter_vector residual_derivative (const std::pair<input_vector, double>& data, const parameter_vector& params)
{
    parameter_vector der;

    const double p0 = params(0);
    const double p1 = params(1);

    const double i0 = data.first(0);

    const double temp = -p0 * exp( - i0 * i0 / ( p1 * p1 ) ) * 2.0 * i0 / ( p1 * p1 ) + 0.2;

    der(0) = i0*2*temp;

    return der;
}

#if 0
typedef matrix<double,3,1> input_vector;
typedef matrix<double,2,1> parameter_vector;

static double model (const input_vector& input, const parameter_vector& params)
{
    const double p0 = params(0);
    const double p1 = params(1);

    const double i0 = input(0);
    const double i1 = input(1);
    const double i2 = input(2);

    const double temp = p0*i0 + p1*i1 + i2;

    return temp*temp;
}

// This function is the "residual" for a least squares problem.   It takes an input/output
// pair and compares it to the output of our model and returns the amount of error.  The idea
// is to find the set of parameters which makes the residual small on all the data pairs.
static double residual (const std::pair<input_vector, double>& data, const parameter_vector& params)
{
    return model(data.first, params) - data.second;
}

// This function is the derivative of the residual() function with respect to the parameters.
static parameter_vector residual_derivative (const std::pair<input_vector, double>& data, const parameter_vector& params)
{
    parameter_vector der;

    const double p0 = params(0);
    const double p1 = params(1);

    const double i0 = data.first(0);
    const double i1 = data.first(1);
    const double i2 = data.first(2);

    const double temp = p0*i0 + p1*i1 + i2;

    der(0) = i0*2*temp;
    der(1) = i1*2*temp;

    return der;
}
#endif
// ----------------------------------------------------------------------------------------
CLeastSquare::CLeastSquare()
{

}

void CLeastSquare::Initialize(void)
{
    m_dCs = 17.4;
    m_dCv = 12.2;

    CDatabase::GetInstance()->SetDsParameterData( 0, m_dCv);
    CDatabase::GetInstance()->SetDsParameterData( 1, m_dCs);

    inputArray.clear();
    outputArray.clear();
}

int CLeastSquare::Estimate( int nTick, double dAccX )
{
    int nSize = inputArray.size();
    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();

    if(nSize >= 1200)
    {
        inputArray.dequeue();
        outputArray.dequeue();
    }

    double dPosX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_X);
    double dPrecedX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_PRECED_X);

    double dGap = dPrecedX - dPosX;

    inputArray.enqueue( dGap );
    outputArray.enqueue( dAccX );

    nSize = inputArray.size();

    //! Set two parameters : Cv, Cs
    parameter_vector params = randm(2,1);
    params(0) = m_dCv;
    params(1) = m_dCs;
    //cout << "params: " << trans(params) << endl;

    // Now let's generate a bunch of input/output pairs according to our model.
    std::vector<std::pair<input_vector, double> > data_samples;
    input_vector input;



    for (int t = 0; t < nSize; ++t)
    {
        input = randm(1,1);
        input(0) = inputArray[t];
        const double output = outputArray[t];

        // save the pair
        data_samples.push_back(make_pair(input, output));
    }


    int nCheck = nTick % 100;
    if(nCheck == 0)
    {
        // Before we do anything, let's make sure that our derivative function defined above matches
        // the approximate derivative computed using central differences (via derivative()).
        // If this value is big then it means we probably typed the derivative function incorrectly.
        //cout << "derivative error: " << length(residual_derivative(data_samples[0], params) - derivative(residual)(data_samples[0], params) ) << endl;
        length(residual_derivative(data_samples[0], params) - derivative(residual)(data_samples[0], params) );



        // Now let's use the solve_least_squares_lm() routine to figure out what the
        // parameters are based on just the data_samples.
        parameter_vector x;
        //x = 1;
        x(0) = m_dCv;
        x(1) = m_dCs;

        //cout << "Use Levenberg-Marquardt/quasi-newton hybrid" << endl;
        // This version of the solver uses a method which is appropriate for problems
        // where the residuals don't go to zero at the solution.  So in these cases
        // it may provide a better answer.
        solve_least_squares(objective_delta_stop_strategy(1e-7).be_verbose(),
                            residual,
                            residual_derivative,
                            data_samples,
                            x);

        m_dCv = x(0);
        m_dCs = x(1);
    }

    m_dCv = qAbs(m_dCv);
    m_dCs = qAbs(m_dCs);

    //! Save estimated parameters in database
    CDatabase::GetInstance()->SetDsParameterData( 0, m_dCv);
    CDatabase::GetInstance()->SetDsParameterData( 1, m_dCs);

    //qDebug() << "leastsquare.cpp @ t=" << nTick << " : omega=" << m_dCv << ", sigma=" << m_dCs;

    data_samples.clear();

    return 0;
}

int CLeastSquare::Estimate(int nTick)
{
#if 0
    static double dPreXg = 0.0;
    static double dPreDotXg = 0.0;
    static double dPreDDotXg = 0.0;

    int nCurrentTrial = CDatabase::GetInstance()->GetCurrentTrial();

    double dTargetX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_X);
    double dTargetVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, DS_OWN_V);
    double dTargetAx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, 13);

    double dPrecedX = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, 15);
    double dPrecedVx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, 18);
    double dPrecedAx = CDatabase::GetInstance()->GetData(DS, nCurrentTrial, nTick, 19);

    double dDeltaV = dPrecedVx - dTargetVx;
    double dDeltaS = dPrecedX - dTargetX;
    double dDeltaA = dPrecedAx - dTargetAx;


    //! Initialize static data
    if(nTick == 0)
    {
        dPreXg = 0.0;
        dPreDotXg = 0.0;
        dPreDDotXg = 0.0;
    }


    double dCoefficientAlpha = 0.8;

    double dXg = (1.0 - dCoefficientAlpha) * dPreXg + dCoefficientAlpha * dDeltaS;
    double dDotXg = (1.0 - dCoefficientAlpha) * dPreDotXg + dCoefficientAlpha * dDeltaV;
    double dDDotXg = (1.0 - dCoefficientAlpha) * dPreDDotXg + dCoefficientAlpha * dDeltaA; // Measured value

    double dEstimatedDDotXg = dDeltaA + m_dCv * (dDeltaV - dDotXg) + m_dCs * (dDeltaS - dXg); // Esitimated value
    dEstimatedDDotXg *= dEstimatedDDotXg;

    int nSize = inputArray0.size();

    if(nSize >= 1200)
    {
        inputArray0.dequeue();
        inputArray1.dequeue();
        inputArray2.dequeue();
        outputArray.dequeue();
    }

    inputArray0.enqueue(dDeltaV - dDotXg);
    inputArray1.enqueue(dDeltaS - dXg);
    inputArray2.enqueue(dDeltaA);

    outputArray.enqueue(dDDotXg*dDDotXg);

    nSize = inputArray0.size();

    //! Set two parameters : Cv, Cs
    parameter_vector params = randm(2,1);
    params(0) = m_dCv;
    params(1) = m_dCs;
    //cout << "params: " << trans(params) << endl;

    // Now let's generate a bunch of input/output pairs according to our model.
    std::vector<std::pair<input_vector, double> > data_samples;
    input_vector input;



    for (int t = 0; t < nSize; ++t)
    {
        input = randm(3,1);
        input(0) = inputArray0[t];
        input(1) = inputArray1[t];
        input(2) = inputArray2[t];
        const double output = outputArray[t];

        // save the pair
        data_samples.push_back(make_pair(input, output));
    }


    int nCheck = nTick % 100;
    if(nCheck == 0)
    {
        // Before we do anything, let's make sure that our derivative function defined above matches
        // the approximate derivative computed using central differences (via derivative()).
        // If this value is big then it means we probably typed the derivative function incorrectly.
        cout << "derivative error: " << length(residual_derivative(data_samples[0], params) -
                                               derivative(residual)(data_samples[0], params) ) << endl;



        // Now let's use the solve_least_squares_lm() routine to figure out what the
        // parameters are based on just the data_samples.
        parameter_vector x;
        x = 1;

        cout << "Use Levenberg-Marquardt/quasi-newton hybrid" << endl;
        // This version of the solver uses a method which is appropriate for problems
        // where the residuals don't go to zero at the solution.  So in these cases
        // it may provide a better answer.
        solve_least_squares(objective_delta_stop_strategy(1e-7).be_verbose(),
                            residual,
                            residual_derivative,
                            data_samples,
                            x);

        // Now x contains the solution.  If everything worked it will be equal to params.
        cout << "inferred parameters: "<< trans(x) << endl;
        cout << "output: " << dDDotXg*dDDotXg << endl;
        cout << "solution error:      "<< dDDotXg*dDDotXg - dEstimatedDDotXg << endl;

        m_dCv = x(0);
        m_dCs = x(1);
    }

    m_dCv = qAbs(m_dCv);
    m_dCs = qAbs(m_dCs);

    //! Save estimated parameters in database
    CDatabase::GetInstance()->SetDsParameterData(nCurrentTrial, nTick, 0, m_dCv);
    CDatabase::GetInstance()->SetDsParameterData(nCurrentTrial, nTick, 1, m_dCs);

    double dDampingRatio = m_dCv / (2.0 * qSqrt(m_dCs));

    if(isNaN(dDampingRatio))
        dDampingRatio = 0.0;

    CDatabase::GetInstance()->SetDsParameterData(nCurrentTrial, nTick, 2, dDampingRatio);


    dPreXg = dXg;
    dPreDotXg = dDotXg;
    dPreDDotXg = dDDotXg;

    data_samples.clear();
#endif


    return 0;
}

#if 0
int CLeastSquare::Estimate(void)
{
    // randomly pick a set of parameters to use in this example
    const parameter_vector params = 1*randm(3,1);
    cout << "params: " << trans(params) << endl;

    // Now let's generate a bunch of input/output pairs according to our model.
    std::vector<std::pair<input_vector, double> > data_samples;
    input_vector input;
    for (int i = 0; i < 1000; ++i)
    {
        input = 1*randm(2,1);
        const double output = model(input, params);

        // save the pair
        data_samples.push_back(make_pair(input, output));
    }

    // Before we do anything, let's make sure that our derivative function defined above matches
    // the approximate derivative computed using central differences (via derivative()).
    // If this value is big then it means we probably typed the derivative function incorrectly.
    cout << "derivative error: " << length(residual_derivative(data_samples[0], params) -
                                           derivative(residual)(data_samples[0], params) ) << endl;

    // Now let's use the solve_least_squares_lm() routine to figure out what the
    // parameters are based on just the data_samples.
    parameter_vector x;
    x = 1;

    cout << "Use Levenberg-Marquardt/quasi-newton hybrid" << endl;
    // This version of the solver uses a method which is appropriate for problems
    // where the residuals don't go to zero at the solution.  So in these cases
    // it may provide a better answer.
    solve_least_squares(objective_delta_stop_strategy(1e-7).be_verbose(),
                        residual,
                        residual_derivative,
                        data_samples,
                        x);

    // Now x contains the solution.  If everything worked it will be equal to params.
    cout << "inferred parameters: "<< trans(x) << endl;
    cout << "solution error:      "<< length(x - params) << endl;

    return 0;
}
#endif

bool CLeastSquare::isNaN(double x)
{
    return x != x;
}
