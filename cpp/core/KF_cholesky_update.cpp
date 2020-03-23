#include "KF_cholesky_update.h"

void KF_cholesky_update(Vector2f &x, Matrix2f &P,Vector2f v,Matrix2f R,Matrix2f H)
{
    Matrix2f PHt = P*H.transpose();
    Matrix2f S = H*PHt + R;
    
    S = (S+S.transpose()) * 0.5; //make symmetric
    Matrix2f SChol = S.llt().matrixL();
    SChol.transpose();
    SChol.conjugate();

    Matrix2f SCholInv = SChol.inverse(); //tri matrix
    Matrix2f W1 = PHt * SCholInv;
    Matrix2f W = W1 * SCholInv.transpose();

    x = x + W*v;
    P = P - W1*W1.transpose();
}
