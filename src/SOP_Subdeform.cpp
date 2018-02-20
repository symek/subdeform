#include <unordered_map>
#include <memory>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>

#include <UT/UT_DSOVersion.h>
#include <GU/GU_Detail.h>
#include <OP/OP_Operator.h>
#include <OP/OP_AutoLockInputs.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_Include.h>
#include <PRM/PRM_SpareData.h>

#include "utils.hpp"
#include "SOP_Subdeform.hpp"

using namespace subdeform;

void
newSopOperator(OP_OperatorTable *table)
{
    table->addOperator(new OP_Operator(
        "subdeform",
        "Subspace Deform",
        SOP_Subdeform::myConstructor,
        SOP_Subdeform::myTemplateList,
        1,
        1,
        0));
}

const char * subspacematrix_help = "File with subspace matrix generated by \
subspace command like utility from rest pose and deformation samples.";

static PRM_Name  deformChoices[] = {
    PRM_Name("0", "Orthogonal"),
    PRM_Name("1", "Principal"),
    PRM_Name(0)
};

static PRM_ChoiceList  deformMenu(PRM_CHOICELIST_SINGLE, deformChoices);

static PRM_Name names[] = {
    PRM_Name("subspacematrix",   "Subspace file"),
    PRM_Name("deformmode",       "Deform mode"),
    PRM_Name("strength",         "Strength"),
};

PRM_Template
SOP_Subdeform::myTemplateList[] = {
    
    PRM_Template(PRM_STRING,    1, &PRMgroupName, 0, &SOP_Node::pointGroupMenu, 0, 0, \
        SOP_Node::getGroupSelectButton(GA_GROUP_POINT)),

    PRM_Template(PRM_PICFILE,   1, &names[0], 0, 0, 0, SOP_Subdeform::markDirty, 
        &PRM_SpareData::fileChooserModeRead, 0, subspacematrix_help), // subspace matrix file

    PRM_Template(PRM_ORD,       1, &names[1], 0, &deformMenu, 0, 0, 0, 0, 0),
    PRM_Template(PRM_FLT_LOG,   1, &names[2], PRMoneDefaults, 0, 0, 0, 0, 0, 0),
    PRM_Template(),
};


OP_Node *
SOP_Subdeform::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new SOP_Subdeform(net, name, op);
}

SOP_Subdeform::SOP_Subdeform(OP_Network *net, const char *name, OP_Operator *op)
    : SOP_Node(net, name, op), myGroup(NULL)
{
   
    mySopFlags.setManagesDataIDs(true);
}

SOP_Subdeform::~SOP_Subdeform() {}

OP_ERROR
SOP_Subdeform::cookInputGroups(OP_Context &context, int alone)
{
    return cookInputPointGroups(
        context, // This is needed for cooking the group parameter, and cooking the input if alone.
        myGroup, // The group (or NULL) is written to myGroup if not alone.
        alone,   // This is true iff called outside of cookMySop to update handles.
                 // true means the group will be for the input geometry.
                 // false means the group will be for gdp (the working/output geometry).
        true,    // (default) true means to set the selection to the group if not alone and the highlight flag is on.
        0,       // (default) Parameter index of the group field
        -1,      // (default) Parameter index of the group type field (-1 since there isn't one)
        true,    // (default) true means that a pointer to an existing group is okay; false means group is always new.
        false,   // (default) false means new groups should be unordered; true means new groups should be ordered.
        true,    // (default) true means that all new groups should be detached, so not owned by the detail;
                 //           false means that new point and primitive groups on gdp will be owned by gdp.
        0        // (default) Index of the input whose geometry the group will be made for if alone.
    );
}


OP_ERROR
SOP_Subdeform::cookMySop(OP_Context &context)
{
    OP_AutoLockInputs inputs(this);
    if (inputs.lock(context) >= UT_ERROR_ABORT)
        return error();
    
    fpreal t = context.getTime();
    duplicatePointSource(0, context);
    // should we care about the cost? (this won't change most of the time)
    m_delta.conservativeResize(gdp->getNumPoints()*3);
   
    /// UI
    UT_String subspace_file, deformmode_str;
    SUBSPACEMATRIX(subspace_file);
    DEFORMMODE(deformmode_str);
    const float strength  = STRENGTH(t);
    const int deform_mode = atoi(deformmode_str.buffer());

    if (error() >= UT_ERROR_ABORT)
        return error();

    // (Re)Init matrices...
    if ((subspace_file.compare(m_matrix_file)) || m_needs_init) {
        
        if(!read_matrix(subspace_file.c_str(), m_matrix)) {
            addError(SOP_MESSAGE, "Failed to load the matrix file.");
            return error();
        }
        DEBUG_PRINT("New matrix read: %s\n", m_matrix_file.c_str());
        m_matrix_file = UT_String(subspace_file.c_str(), true);
        m_transposed = m_matrix.transpose();
        m_qrmatrix    = nullptr;
        m_needs_init  = false;
    }

    // (A) get weights from orthogonalized shape matrix 
    if (deform_mode == deformation_space::ORTHO) {
        m_weights.conservativeResize(m_matrix.cols());
        GA_Attribute * rest = gdp->findFloatTuple(GA_ATTRIB_POINT, "rest", 3);
        if (!rest) {
            addWarning(SOP_MESSAGE, "We need rest attribute to proceed."); 
            return error();
        }
        if(!position_delta(gdp, m_delta)) {
            addWarning(SOP_MESSAGE, "Can't compute delta frame.");
            return error();
        }
        // scalar product of delta and Q's columns:
        // Get weights out of this: 
        if(m_qrmatrix == nullptr) {
            DEBUG_PRINT("Building new QRMatrix... %s\n", "");
            m_qrmatrix = std::move(QRMatrixPtr(new QRMatrix(m_matrix)));
        }
        Matrix weights_mat = m_delta.asDiagonal() * m_qrmatrix->matrixQR();
        m_weights = std::move(weights_mat.colwise().sum());
        apply_displacement(m_matrix, m_weights, strength, gdp);

    } else if (deform_mode == deformation_space::PCA) {

        GA_ROHandleV3 rest_h(gdp->findFloatTuple(GA_ATTRIB_POINT, "rest", 3));
        Vector subpos(gdp->getNumPoints()*3);

         if(!position_delta(gdp, m_delta)) {
            addWarning(SOP_MESSAGE, "Can't compute delta frame.");
            return error();
        }
        // 
        // NOTE: those parenthiss are importants
        m_delta  = m_matrix * (m_transposed * m_delta);
        GA_Offset ptoff;
        GA_FOR_ALL_PTOFF(gdp, ptoff) {
            const GA_Index ptidx  = gdp->pointIndex(ptoff);
            const UT_Vector3 old  = gdp->getPos3(ptoff);
            const Vector & v      = m_delta.segment<3>(ptidx*3);
            const UT_Vector3 disp(v.data());
            gdp->setPos3(ptoff, old + disp * strength);
        }
    
    }

    // If we've modified P, and we're managing our own data IDs,
    // we must bump the data ID for P.
    if (!myGroup || !myGroup->isEmpty())
        gdp->getP()->bumpDataId();

    return error();
}
