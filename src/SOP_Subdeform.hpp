#pragma once 
#include <SOP/SOP_Node.h>

namespace subdeform {

#ifndef NDEBUG
#define DEBUG_PRINT(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) do {} while (0)
#endif

//#define SPARSE

enum deformation_space {
    ORTHO,
    PCA,
};

inline bool position_delta(const GU_Detail * gdp, Vector & delta) {
    GA_ROHandleV3 rest_h(gdp->findFloatTuple(GA_ATTRIB_POINT, "rest", 3));
    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(gdp, ptoff) {
        const GA_Index ptidx  = gdp->pointIndex(ptoff);
        const UT_Vector3 pos  = gdp->getPos3(ptoff);
        const UT_Vector3 rest = rest_h.get(ptoff);
        delta(3*ptidx + 0) = pos.x() - rest.x();
        delta(3*ptidx + 1) = pos.y() - rest.y();
        delta(3*ptidx + 2) = pos.z() - rest.z();
    } 
    return true;   
}

inline void apply_displacement(const Matrix & matrix, 
    const Vector & weights, const float strength, GU_Detail * gdp) {
    GA_Offset ptoff;
    GA_FOR_ALL_PTOFF(gdp, ptoff) {
        const GA_Index ptidx  = gdp->pointIndex(ptoff);
        UT_Vector3 disp(0,0,0);
        for(int col=0; col<matrix.cols(); ++col) {
            const float xd = matrix(3*ptidx + 0, col);
            const float yd = matrix(3*ptidx + 1, col);
            const float zd = matrix(3*ptidx + 2, col);
            const float w  = weights(col); 
            disp += UT_Vector3(xd, yd, zd) * w ;
        }
        const UT_Vector3 rest = gdp->getPos3(ptoff);
        gdp->setPos3(ptoff, rest - (disp * strength));
    }
}

class SOP_Subdeform : public SOP_Node
{
public:
    typedef Eigen::VectorXd              DeltaVector;
    typedef Eigen::HouseholderQR<Matrix> QRMatrix;
    typedef std::unique_ptr<QRMatrix>    QRMatrixPtr;
    typedef std::unique_ptr<Eigen::VectorXd> WeightsVector;
    SOP_Subdeform(OP_Network *net, const char *name, OP_Operator *op);
    virtual ~SOP_Subdeform();
    /// Mark internal storage needs to be recreated (m_matrix, ...)
    static int markDirty(void *data, int, fpreal, const PRM_Template *) { 
        SOP_Subdeform *node = static_cast<SOP_Subdeform*>(data);
        node->m_needs_init = true;
        return 1;
    }
    
    static PRM_Template      myTemplateList[];
    static OP_Node      *myConstructor(OP_Network*, const char *,
                                OP_Operator *);

    /// This method is created so that it can be called by handles.  It only
    /// cooks the input group of this SOP.  The geometry in this group is
    /// the only geometry manipulated by this SOP.
    virtual OP_ERROR         cookInputGroups(OP_Context &context, 
                        int alone = 0);

protected:
    /// Method to cook geometry for the SOP
    virtual OP_ERROR         cookMySop(OP_Context &context);

private:
    void    getGroups(UT_String &str)         { evalString(str, "group", 0, 0); }
    void    SUBSPACEMATRIX(UT_String &str)    { evalString(str, "subspacematrix", 0, 0); }
    void    DEFORMMODE(UT_String &str)        { evalString(str, "deformmode", 0, 0); }
    fpreal  STRENGTH(fpreal t)                { return evalFloat("strength", 0, t); }

    /// This is the group of geometry to be manipulated by this SOP and cooked
    /// by the method "cookInputGroups".
    const GA_PointGroup *myGroup;
    Matrix        m_matrix;
    Matrix        m_transposed;
    DeltaVector   m_delta;
    QRMatrixPtr   m_qrmatrix = nullptr;
    DeltaVector   m_weights;
    bool          m_needs_init = true;

};



} // End HDK_Sample namespace

