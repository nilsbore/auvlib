#include <sparse_gp/sparse_gp.h>

#include <ctime>
#include <stdio.h> // until we removed the old debug printouts
#include <iostream>

#include <sparse_gp/octave_convenience.h>

#define DEBUG 0

using namespace Eigen;

/*sparse_gp::sparse_gp(int capacity, double s0, double sigmaf, double l) :
    capacity(capacity), s20(s0), sigmaf_sq(sigmaf), l_sq(l),
    eps_tol(1e-5f), current_size(0), total_count(0) // 1e-6f
{

}*/

/*sparse_gp::sparse_gp(base_kernel* kernel, int capacity, double s0) :
    kernel(kernel), capacity(capacity), s20(s0),
    eps_tol(1e-5f), current_size(0), total_count(0) // 1e-6f
{

}*/

template <class Kernel, class Noise>
sparse_gp<Kernel, Noise>::sparse_gp(int capacity, double s0) :
    kernel(), noise(s0), capacity(capacity), s20(s0),
    eps_tol(1e-6f), current_size(0), total_count(0) // 1e-6f
{

}

template <class Kernel, class Noise>
int sparse_gp<Kernel, Noise>::size()
{
    return current_size;
}

// shuffle the data so that all neighbouring points aren't added at once
template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::shuffle(std::vector<int>& ind, int n)
{
    ind.resize(n);
    for (int i = 0; i < n; ++i) {
        ind[i] = i;
    }
    int temp, r;
    for (int i = n-1; i > 0; --i) {
        r = rand() % i;
        temp = ind[i];
        ind[i] = ind[r];
        ind[r] = temp;
    }
}

//Add a chunk of data
template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::add_measurements(const MatrixXd& X,const VectorXd& y)
{
    std::vector<int> ind;
    shuffle(ind, y.rows());

    /*std::cout << "[ ";
    for (int i = 0; i < X.rows(); ++i) {
        std::cout << X.row(ind[i]);
        if (i < X.rows() - 1) {
            std::cout << " ;" << std::endl;
        }
        else {
            std::cout << " ]" << std::endl;
        }
    }

    std::cout << "[ ";
    for (int i = 0; i < y.rows(); ++i) {
        std::cout << y(ind[i]) << " ";
    }
    std::cout << "]" << std::endl;*/

    for (int i = 0; i < X.rows(); i++) {
        add(X.row(ind[i]).transpose(), y(ind[i])); // row transpose
    }

}

//Add this input and output to the GP
template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::add(const VectorXd& X, double y)
{

    total_count++;
    if (DEBUG) {
        printf("\t\tsparse_gp adding %dth point to sparse_gp (current_size = %d):",total_count,current_size);
    }
    //double kstar = kernel_function(X, X);
    double kstar = kernel.kernel_function(X, X);

    if (current_size == 0){//First point is easy
        alpha.resize(1);
        C.resize(1, 1);
        Q.resize(1, 1);
        //Equations 2.46 with q, r, and s collapsed
        alpha(0) = y / (kstar + s20); // resize alpha?
        C(0, 0) = -1.0f / (kstar + s20);
        Q(0, 0) = 1.0f / kstar; // K_1^-1

        current_size = 1;
        BV = X;
        if (DEBUG) {
            printf("First\n");
        }
    }
    else {   //We already have data
        //perform the kernel
        VectorXd k;
        // this could be done a lot faster if we had a cc_fast for BV
        construct_covariance(k, X, BV);

        double m = alpha.transpose()*k;
        double s2 = kstar + k.transpose()*C*k;

        if (s2 < 1e-12f) {//For numerical stability..from Csato's Matlab code?
            if (DEBUG) {
                printf("s2 %lf ",s2);
            }
            //s2 = 1e-12f; // this is where the nan comes from
        }

        //Update scalars
        //page 33 - Assumes Gaussian noise, no assumptions on kernel?
        //double r = -1.0f / (s20 + s2);
        double r = noise.dx2_ln(y, m, s2); // -1.0f / (s20 + s2);
        //printf("out %d m %d r %f\n",out.Nrows(),m.Ncols(),r);
        //double q = -r*(y - m);
        double q = noise.dx_ln(y, m, s2); // -r*(y - m);

        //projection onto current BV
        VectorXd e_hat = Q*k;//Appendix G, section c
        //residual length
        // below equation 3.5

        double gamma = kstar - k.transpose()*e_hat;//Ibid

        if (gamma < 1e-12f) {//Numerical instability?
            if (DEBUG) {
                printf(" Gamma %lf ",gamma);
            }
            gamma = 0;
        }

        // these if statements should probably be the other way around, gamma < eps_tol ==> full update
        // No, sparse and full seem to be interchanged in Csato's thesis
        if (gamma < eps_tol && capacity != -1) {//Nearly singular, do a sparse update (e_tol), makes sure Q not singular
            if (DEBUG) {
                printf("Sparse %lf \n",gamma);
            }
            double eta = 1/(1 + gamma*r);//Ibid
            VectorXd s_hat = C*k + e_hat;//Appendix G section e
            alpha += s_hat*(q * eta);//Appendix G section f
            C += r*eta*s_hat*s_hat.transpose();//Ibid
        }
        else { //Full update
            if (DEBUG) {
                printf("Full!\n");
            }

            //s is of length N+1
            VectorXd s(k.rows() + 1);
            s.head(k.rows()) = C*k; // + e_hat, no only when sparse, OK!
            s(s.rows() - 1) = 1.0f; // shouldn't this be e_hat instead?

            //Add a row to alpha
            alpha.conservativeResize(alpha.rows() + 1);
            alpha(alpha.rows() - 1) = 0;
            //Update alpha
            alpha += q*s;//Equations 2.46

            //Add Row and Column to C
            C.conservativeResize(C.rows() + 1, C.cols() + 1);
            C.row(C.rows() - 1).setZero();
            C.col(C.cols() - 1).setZero();
            //Update C
            C += r*s*s.transpose();//Ibid

            //Save the data, N++
            BV.conservativeResize(BV.rows(), current_size + 1);
            BV.col(current_size) = X;
            current_size++;

            //Add row and column to Gram Matrix
            Q.conservativeResize(Q.rows() + 1, Q.cols() + 1);
            Q.row(Q.rows() - 1).setZero();
            Q.col(Q.cols() - 1).setZero();

            //Add one more to ehat
            e_hat.conservativeResize(e_hat.rows() + 1);
            e_hat(e_hat.rows() - 1) = -1.0f;
            //Update gram matrix
            Q += 1.0f/gamma*e_hat*e_hat.transpose();//Equation 3.5

        }

        //Delete BVs if necessary...maybe only 2 per iteration?
        while (current_size > capacity && capacity > 0) { //We're too big!
            double minscore = 0, score;
            int minloc = -1;
            //Find the minimum score
            for (int i = 0; i < current_size; i++) {
                // Equation 3.26
                score = alpha(i)*alpha(i)/(Q(i, i) + C(i, i)); // SumSquare()?
                if (i == 0 || score < minscore) {
                    minscore = score;
                    minloc = i;
                }
            }
            //Delete it
            delete_bv(minloc);
            if (DEBUG) {
                printf("Deleting for size %d\n", current_size);
            }
        }

        //Delete for geometric reasons - Loop?
        double minscore = 0, score;
        int minloc = -1;
        while (minscore < 1e-9f && current_size > 1) {
            for (int i = 0; i < current_size; i++) {
                score = 1.0f/Q(i, i);
                if (i == 0 || score < minscore) {
                    minscore = score;
                    minloc = i;
                }
            }
            if (minscore < 1e-9f) {
                delete_bv(minloc);
                if (DEBUG) {
                    printf("Deleting for geometry\n");
                }
            }
        }

    }
    if (std::isnan(C(0, 0))) {
        printf("sparse_gp::C has become Nan after %d inducing points\n", current_size);
		exit(0);
    }

}

//Delete a BV.  Very messy
template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::delete_bv(int loc)
{
    //First swap loc to the last spot
    double alphastar = alpha(loc);
    alpha(loc) = alpha(alpha.rows() - 1);
    alpha.conservativeResize(alpha.rows() - 1);

    //Now C
    double cstar = C(loc, loc);
    VectorXd Cstar = C.col(loc);
    Cstar(loc) = Cstar(Cstar.rows() - 1);
    Cstar.conservativeResize(Cstar.rows() - 1);

    VectorXd Crep = C.col(C.cols() - 1);
    Crep(loc) = Crep(Crep.rows() - 1);
    C.row(loc) = Crep.transpose();
    C.col(loc) = Crep;
    C.conservativeResize(C.rows() - 1, C.cols() - 1);

    //and Q
    double qstar = Q(loc, loc);
    VectorXd Qstar = Q.col(loc);
    Qstar(loc) = Qstar(Qstar.rows() - 1);
    Qstar.conservativeResize(Qstar.rows() - 1);
    VectorXd Qrep = Q.col(Q.cols() - 1);
    Qrep(loc) = Qrep(Qrep.rows() - 1);
    Q.row(loc) = Qrep.transpose();
    Q.col(loc) = Qrep;
    Q.conservativeResize(Q.rows() - 1, Q.cols() - 1);

    //Ok, now do the actual removal  Appendix G section g
    //VectorXd qc = (Qstar + Cstar)/(qstar + cstar);
    alpha -= alphastar/(qstar + cstar)*(Qstar + Cstar);
    C += (Qstar * Qstar.transpose()) / qstar -
            ((Qstar + Cstar)*(Qstar + Cstar).transpose()) / (qstar + cstar);
    Q -= (Qstar * Qstar.transpose()) / qstar;

    //And the BV
    BV.col(loc) = BV.col(BV.cols() - 1);
    BV.conservativeResize(BV.rows(), BV.cols() - 1);

    current_size--;
}


//Predict on a chunk of data.
template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::predict_measurements(VectorXd& f_star, const MatrixXd& X_star, VectorXd& sigconf, bool conf)
{
    //printf("sparse_gp::Predicting on %d points\n",in.Ncols());
    f_star.resize(X_star.rows());
    sigconf.resize(X_star.rows());
    for (int c = 0; c < X_star.rows(); c++) {
        f_star(c) = predict(X_star.row(c).transpose(), sigconf(c), conf); // row transpose
    }
}

//Predict the output and uncertainty for this input.
// Make this work for several test inputs
template <class Kernel, class Noise>
double sparse_gp<Kernel, Noise>::predict(const VectorXd& X_star, double& sigma, bool conf)
{
    //double kstar = kernel_function(X_star, X_star);
    double kstar = kernel.kernel_function(X_star, X_star);
    VectorXd k;
    construct_covariance(k, X_star, BV);

    double f_star;
    if (current_size == 0) {
        f_star = 0; // gaussian around 0
        sigma = kstar + s20;
        if (DEBUG) {
            printf("No training points added before prediction\n");
        }
    }
    else {
        f_star = alpha.transpose()*k; //Page 33
        sigma = s20 + kstar + k.transpose()*C*k; //Ibid..needs s2 from page 19
        // here we predict y and not f, adding s20
    }

    if (sigma < 0) { //Numerical instability?
        printf("sparse_gp:: sigma (%lf) < 0!\n",sigma);
        sigma = 0;
    }

    //Switch to a confidence (0-100)
    if (conf) {
        //Normalize to one
        sigma /= kstar + s20;
        //switch diretion
        sigma = 100.0f*(1.0f - sigma);
    }
    else {
        sigma = sqrt(sigma);
    }

    return f_star;
}


//Log probability of this data
//Sigma should really be a matrix...
//Should this use Q or -C?  Should prediction use which?
template <class Kernel, class Noise>
double sparse_gp<Kernel, Noise>::neg_log_likelihood(const VectorXd& X, double y)
{
    static const double logsqrt2pi = 0.5f*log(2.0f*M_PI);
    double sigma;
    double mu;

    //This is pretty much prediction
    double kstar = kernel.kernel_function(X, X);
    VectorXd k;
    construct_covariance(k, X, BV);
    if (current_size == 0) {
        sigma = kstar + s20;
        mu = 0;
    }
    else {
        mu = k.transpose()*alpha;//Page 33
        sigma = s20 + kstar + k.transpose()*C*k;//Ibid..needs s2 from page 19
        //printf("Making sigma: %lf %lf %lf %lf\n",s20,kstar,k(1),C(1,1));
    }
    double cent2 = (y - mu)*(y - mu);

    // in eigen, we have to use decomposition to get log determinant
    long double log_det_cov = log(sigma);

    long double lp = logsqrt2pi + 0.5f*log_det_cov + 0.5f*cent2/sigma;
    //printf("\tCalculating log prob %Lf, Sigma = %lf, cent = %lf\n",lp,sigma,cent2);
    //if(C.Nrows()==1)
    //printf("\t\tC has one entry: %lf, k = %lf\n",C(1,1),k(1));
    return lp;
}

template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::compute_likelihoods(VectorXd& l, const MatrixXd& X, const VectorXd& y)
{
    l.resize(X.rows());
    for (int i = 0; i < X.rows(); ++i) {
        l(i) = likelihood(X.row(i).transpose(), y(i));
    }
}

template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::compute_neg_log_likelihoods(VectorXd& l, const MatrixXd& X, const VectorXd& y)
{
    l.resize(X.rows());
    for (int i = 0; i < X.rows(); ++i) {
        l(i) = neg_log_likelihood(X.row(i).transpose(), y(i));
    }
}

// likelihood, without log for derivatives
template <class Kernel, class Noise>
double sparse_gp<Kernel, Noise>::likelihood(const Vector2d& x, double y)
{
    //This is pretty much prediction
    //double kstar = kernel_function(x, x);
    double kstar = kernel.kernel_function(x, x);
    VectorXd k;
    construct_covariance(k, x, BV);
    double mu;
    double sigma;
    if (current_size == 0) { // no measurements, only prior gaussian around 0
        mu = 0.0f;
        sigma = kstar + s20; // maybe integrate this into likelihood_dx also
    }
    else {
        mu = k.transpose()*alpha; //Page 33
        sigma = s20 + kstar + k.transpose()*C*k;//Ibid..needs s2 from page 19
    }
    return 1.0f/sqrt(2.0f*M_PI*sigma)*exp(-0.5f/sigma*(y - mu)*(y - mu));
}

template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::compute_derivatives_fast(MatrixXd& dX, const MatrixXd& X, const VectorXd& y)
{
    if (X.rows() == 0) {
        dX.resize(0, 3);
        return;
    }
    MatrixXd K;
    //construct_covariance_fast(K, X.transpose());
    kernel.construct_covariance_fast(K, X.transpose(), BV);
    ArrayXXd K_dx;
    ArrayXXd K_dy;
    //std::cout << "BV height " << BV.rows() << ", width " << BV.cols() << std::endl;
    //kernels_fast(K_dx, K_dy, X.transpose());
    kernel.kernels_fast(K_dx, K_dy, X.transpose(), BV);
    ArrayXXd CK = C*K;
    ArrayXd sigma_dx = 2.0f*(K_dx*CK).colwise().sum();
    ArrayXd sigma_dy = 2.0f*(K_dy*CK).colwise().sum();
    ArrayXd sigma = (K.array()*CK).colwise().sum() + s20;
    ArrayXd sqrtsigma = sigma.sqrt();
    ArrayXd offset = y - K.transpose()*alpha; // transpose?
    ArrayXd sqoffset = offset*offset;
    ArrayXd exppart = 0.5f*(-0.5f*sqoffset/sigma).exp()/(sigma*sqrtsigma);

    //Array2d firstpart = -sigma_dx;
    ArrayXd secondpart_dx = 2.0f*(K_dx.transpose().matrix()*alpha).array()*offset;
    ArrayXd secondpart_dy = 2.0f*(K_dy.transpose().matrix()*alpha).array()*offset;
    ArrayXd thirdpart_dx = sigma_dx/sigma * sqoffset;
    ArrayXd thirdpart_dy = sigma_dy/sigma * sqoffset;
    dX.resize(X.rows(), 3);
    dX.col(0) = -1.0f*offset*exppart/(sigma*sqrtsigma);
    dX.col(1) = exppart*(-sigma_dx + secondpart_dx + thirdpart_dx);
    dX.col(2) = exppart*(-sigma_dy + secondpart_dy + thirdpart_dy);
}

template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::compute_derivatives(MatrixXd& dX, const MatrixXd& X, const VectorXd& y)
{
    dX.resize(X.rows(), 3);
    Vector3d dx;
    for (int i = 0; i < X.rows(); ++i) {
        likelihood_dx(dx, X.row(i).transpose(), y(i));
        dX.row(i) = dx.transpose();
    }
}

template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::compute_neg_log_derivatives(MatrixXd& dX, const MatrixXd& X, const VectorXd& y)
{
    dX.resize(X.rows(), 3);
    Vector3d dx;
    for (int i = 0; i < X.rows(); ++i) {
        neg_log_likelihood_dx(dx, X.row(i).transpose(), y(i));
        dX.row(i) = dx.transpose();
    }
}

// THIS NEEDS SOME SPEEDUP, PROBABLY BY COMPUTING SEVERAL AT ONCE
// the differential likelihood with respect to x and y
template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::likelihood_dx(Vector3d& dx, const VectorXd& x, double y)
{
    VectorXd k;
    double k_star = kernel.kernel_function(x, x);
    construct_covariance(k, x, BV);
    MatrixXd k_dx;
    MatrixXd k_star_dx;
    //std::cout << "BV height " << BV.rows() << ", width " << BV.cols() << std::endl;
    //kernel_dx(k_dx, x);
    kernel.kernel_dx(k_dx, x, BV);
    //MatrixXd temp = x;
    kernel.kernel_dx(k_star_dx, x, x);
    Array2d sigma_dx = 2.0f*k_dx.transpose()*C*k + k_star_dx.transpose(); // with or without k_star?
    double sigma = s20 + k.transpose()*C*k + k_star;
    double sqrtsigma = sqrt(sigma);
    double offset = y - k.transpose()*alpha;
    double exppart = 0.5f/(sigma*sqrtsigma)*exp(-0.5f/sigma * offset*offset);
    Array2d firstpart = -sigma_dx;
    Array2d secondpart = (2.0f*(k_dx.transpose()*alpha).array()*offset);// +
    Array2d thirdpart = sigma_dx/sigma * offset*offset;
    dx(0) = -1.0f/(sigma*sqrtsigma)*offset*exppart;
    dx.tail<2>() = exppart*(firstpart + secondpart + thirdpart);
    if (std::isnan(dx(0)) || std::isnan(dx(1)) || std::isnan(dx(2))) {
        //breakpoint();
    }

//    VectorXd k;
//    construct_covariance(k, x, BV);
//    dx(0) = k.transpose()*alpha - y;
//    dx.tail<2>().setZero();
}

// NOTE: this is kind of a guess right now, we need to properly
// derive this again to control that it is correct
template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::neg_log_likelihood_dx(Vector3d& dx, const VectorXd& x, double y)
{
    VectorXd k;
    double k_star = kernel.kernel_function(x, x);
    construct_covariance(k, x, BV);
    MatrixXd k_dx;
    MatrixXd k_star_dx;
    kernel.kernel_dx(k_dx, x, BV);
    kernel.kernel_dx(k_star_dx, x, x);
    Array2d sigma_dx = 2.0f*k_dx.transpose()*C*k + k_star_dx.transpose(); // with or without k_star?
    double sigma = s20 + k.transpose()*C*k + k_star;
    double sqrtsigma = sqrt(sigma);
    double offset = y - k.transpose()*alpha;
    //double exppart = 0.5f/(sigma*sqrtsigma)*exp(-0.5f/sigma * offset*offset);
    Array2d firstpart = -sigma_dx;
    Array2d secondpart = (2.0f*(k_dx.transpose()*alpha).array()*offset);// +
    Array2d thirdpart = sigma_dx/sigma * offset*offset;
    dx(0) = -1.0f/sigma*offset;
    dx.tail<2>() = -firstpart - secondpart - thirdpart;
    if (std::isnan(dx(0)) || std::isnan(dx(1)) || std::isnan(dx(2))) {
        //breakpoint();
    }
}

template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::likelihood_dtheta(VectorXd& dtheta, const Vector2d& x, double y)
{
    dtheta.resize(kernel.param_size());
    VectorXd k;
    construct_covariance(k, x, BV);
    MatrixXd k_dtheta;
    kernel.kernel_dtheta(k_dtheta, x, BV);
    dtheta = (alpha.transpose()*k - y)*k_dtheta.transpose()*alpha;
}

// kernel function, to be separated out later
template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::construct_covariance(VectorXd& K, const Vector2d& X, const MatrixXd& Xv)
{
    K.resize(Xv.cols());
    for (int i = 0; i < Xv.cols(); ++i) {
        //K(i) = kernel_function(X, Xv.col(i));
        K(i) = kernel.kernel_function(X, Xv.col(i));
    }
}

// squared exponential coviariance, should use matern instead
/*double sparse_gp::kernel_function(const Vector2d& xi, const Vector2d& xj)
{
    return sigmaf_sq*exp(-0.5f / l_sq * (xi - xj).squaredNorm());
}*/

// linear kernel
/*double sparse_gp::kernel_function(const Vector2d& xi, const Vector2d& xj)
{
    return xi.transpose()*xj;
}*/

// polynomial kernel
/*double sparse_gp::kernel_function(const Vector2d& xi, const Vector2d& xj)
{
  double d = xi.rows();
  double resp = 1;
  double inner = xi.transpose()*xj;
  for (int i = 0; i < scales.Ncols(); i++) {
      resp += pow(inner / (d*scales(i)), i);
  }
  return resp;
}*/

// rbf kernel
/*double sparse_gp::kernel(const Vector2d& xi, const Vector2d& xj)
{
    int d = xi.rows();
    if (d != widths.cols()) {//Expand if necessary
        //printf("RBFKernel:  Resizing width to %d\n",(int)d);
        double wtmp = widths(0);
        widths.resize(d);
        for (int i = 0; i < d; i++) {
            widths(i) = wtmp;
        }
    }
    //I think this bumps up against numerical stability issues.
    return A*exp(-0.5f/d * (SP(a-b,widths.t())).squaredNorm());
}*/

// reset all parameters of gp so that it can be trained again
template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::reset()
{
    total_count = 0;
    current_size = 0;
    alpha.resize(0); // just to empty memory
    C.resize(0, 0);
    Q.resize(0, 0);
    BV.resize(0, 0);
}

// train parameters of kernel using ???
template <class Kernel, class Noise>
void sparse_gp<Kernel, Noise>::train_parameters(const MatrixXd& X, const VectorXd& y)
{
    if (this->size() > 0) {
        if (DEBUG) {
            printf("Not training because process not empty.");
        }
        return;
    }
    int n = kernel.param_size();
    VectorXd delta(n);
    VectorXd deltai(n);
    VectorXd l(X.rows());
    VectorXd l_old(X.rows());
    l.setZero();
    l_old.setZero();
    std::vector<double> ls;
    std::vector<ArrayXXd> dKs;
    dKs.resize(n);
    double step = 1e-4f;
    bool first = true;
    do { // iterate until likelihood difference between runs is small
        reset();
        add_measurements(X, y);
        if (first && BV.cols() < 20) { // DEBUG
            return;
        }
        ArrayXXd dlPdK = -0.5f*(alpha*alpha.transpose() + C.transpose());
        int counter = 0;
        do { // iterate until derivative is small enough
            /*kernel.dKdtheta(dKs, BV);
            for (int i = 0; i < n; ++i) {
                delta(i) = (dlPdK*dKs[i]).sum();
            }*/
            delta.setZero();
            for (int i = 0; i < X.rows(); ++i) {
                likelihood_dtheta(deltai, X.row(i).transpose(), y(i));
                delta += deltai;
            }
            kernel.param()(0) += step*delta(0);
            compute_neg_log_likelihoods(l, X, y);
            std::cout << "Delta norm: " << delta(0) << std::endl;
            ls.push_back(-l.sum());
            std::cout << "L norm: " << l.norm() << std::endl;
            std::cout << "P: " << kernel.param().transpose() << std::endl;
            if (counter > 100) {
                break;
            }
            ++counter;
            first = false;
        }
        while (delta.norm() > 1e-2f);

        octave_convenience oc;
        oc.eval_plot_vector(ls);
        exit(0);

        l_old = l;
        compute_likelihoods(l, X, y);
        ls.push_back(l.norm());
        std::cout << "L norm: " << (l - l_old).norm() << std::endl;
    }
    while ((l - l_old).norm() > 1e0f);
    octave_convenience oc;
    oc.eval_plot_vector(ls);
}
