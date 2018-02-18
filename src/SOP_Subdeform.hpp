#pragma once 
#include <SOP/SOP_Node.h>


namespace subdeform {

#ifndef NDEBUG
#define DEBUG_PRINT(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) do {} while (0)
#endif

enum deformation_space {
    ORTHO,
    PCA,
};


class SOP_Subdeform : public SOP_Node
{
public:
    SOP_Subdeform(OP_Network *net, const char *name, OP_Operator *op);
    virtual ~SOP_Subdeform();

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


    /// This is the group of geometry to be manipulated by this SOP and cooked
    /// by the method "cookInputGroups".
    const GA_PointGroup *myGroup;
    Matrix    m_matrix;
    UT_String m_matrix_file;
    bool      m_needs_init = true;

};



} // End HDK_Sample namespace

