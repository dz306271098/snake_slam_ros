
#include "Binding_pch.h"


using namespace gl;


namespace glbinding
{
Function<void, GLuint> Binding::CallCommandListNV("glCallCommandListNV");
Function<void, GLuint> Binding::CallList("glCallList");
Function<void, GLsizei, GLenum, const void*> Binding::CallLists("glCallLists");
Function<GLenum, GLenum> Binding::CheckFramebufferStatus("glCheckFramebufferStatus");
Function<GLenum, GLenum> Binding::CheckFramebufferStatusEXT("glCheckFramebufferStatusEXT");
Function<GLenum, GLuint, GLenum> Binding::CheckNamedFramebufferStatus("glCheckNamedFramebufferStatus");
Function<GLenum, GLuint, GLenum> Binding::CheckNamedFramebufferStatusEXT("glCheckNamedFramebufferStatusEXT");
Function<void, GLenum, GLboolean> Binding::ClampColor("glClampColor");
Function<void, GLenum, GLboolean> Binding::ClampColorARB("glClampColorARB");
Function<void, ClearBufferMask> Binding::Clear("glClear");
Function<void, GLfloat, GLfloat, GLfloat, GLfloat> Binding::ClearAccum("glClearAccum");
Function<void, GLfixed, GLfixed, GLfixed, GLfixed> Binding::ClearAccumxOES("glClearAccumxOES");
Function<void, GLenum, GLenum, GLenum, GLenum, const void*> Binding::ClearBufferData("glClearBufferData");
Function<void, GLenum, GLenum, GLintptr, GLsizeiptr, GLenum, GLenum, const void*> Binding::ClearBufferSubData(
    "glClearBufferSubData");
Function<void, GLenum, GLint, GLfloat, GLint> Binding::ClearBufferfi("glClearBufferfi");
Function<void, GLenum, GLint, const GLfloat*> Binding::ClearBufferfv("glClearBufferfv");
Function<void, GLenum, GLint, const GLint*> Binding::ClearBufferiv("glClearBufferiv");
Function<void, GLenum, GLint, const GLuint*> Binding::ClearBufferuiv("glClearBufferuiv");
Function<void, GLfloat, GLfloat, GLfloat, GLfloat> Binding::ClearColor("glClearColor");
Function<void, GLint, GLint, GLint, GLint> Binding::ClearColorIiEXT("glClearColorIiEXT");
Function<void, GLuint, GLuint, GLuint, GLuint> Binding::ClearColorIuiEXT("glClearColorIuiEXT");
Function<void, GLfixed, GLfixed, GLfixed, GLfixed> Binding::ClearColorxOES("glClearColorxOES");
Function<void, GLdouble> Binding::ClearDepth("glClearDepth");
Function<void, GLdouble> Binding::ClearDepthdNV("glClearDepthdNV");
Function<void, GLfloat> Binding::ClearDepthf("glClearDepthf");
Function<void, GLclampf> Binding::ClearDepthfOES("glClearDepthfOES");
Function<void, GLfixed> Binding::ClearDepthxOES("glClearDepthxOES");
Function<void, GLfloat> Binding::ClearIndex("glClearIndex");
Function<void, GLuint, GLenum, GLenum, GLenum, const void*> Binding::ClearNamedBufferData("glClearNamedBufferData");
Function<void, GLuint, GLenum, GLenum, GLenum, const void*> Binding::ClearNamedBufferDataEXT(
    "glClearNamedBufferDataEXT");
Function<void, GLuint, GLenum, GLintptr, GLsizeiptr, GLenum, GLenum, const void*> Binding::ClearNamedBufferSubData(
    "glClearNamedBufferSubData");
Function<void, GLuint, GLenum, GLsizeiptr, GLsizeiptr, GLenum, GLenum, const void*> Binding::ClearNamedBufferSubDataEXT(
    "glClearNamedBufferSubDataEXT");
Function<void, GLuint, GLenum, GLint, GLfloat, GLint> Binding::ClearNamedFramebufferfi("glClearNamedFramebufferfi");
Function<void, GLuint, GLenum, GLint, const GLfloat*> Binding::ClearNamedFramebufferfv("glClearNamedFramebufferfv");
Function<void, GLuint, GLenum, GLint, const GLint*> Binding::ClearNamedFramebufferiv("glClearNamedFramebufferiv");
Function<void, GLuint, GLenum, GLint, const GLuint*> Binding::ClearNamedFramebufferuiv("glClearNamedFramebufferuiv");
Function<void, GLint> Binding::ClearStencil("glClearStencil");
Function<void, GLuint, GLint, GLenum, GLenum, const void*> Binding::ClearTexImage("glClearTexImage");
Function<void, GLuint, GLint, GLint, GLint, GLint, GLsizei, GLsizei, GLsizei, GLenum, GLenum, const void*>
    Binding::ClearTexSubImage("glClearTexSubImage");
Function<void, GLenum> Binding::ClientActiveTexture("glClientActiveTexture");
Function<void, GLenum> Binding::ClientActiveTextureARB("glClientActiveTextureARB");
Function<void, GLenum> Binding::ClientActiveVertexStreamATI("glClientActiveVertexStreamATI");
Function<void, ClientAttribMask> Binding::ClientAttribDefaultEXT("glClientAttribDefaultEXT");
Function<GLenum, GLsync, SyncObjectMask, GLuint64> Binding::ClientWaitSync("glClientWaitSync");
Function<void, GLenum, GLenum> Binding::ClipControl("glClipControl");
Function<void, GLenum, const GLdouble*> Binding::ClipPlane("glClipPlane");
Function<void, GLenum, const GLfloat*> Binding::ClipPlanefOES("glClipPlanefOES");
Function<void, GLenum, const GLfixed*> Binding::ClipPlanexOES("glClipPlanexOES");
Function<void, GLbyte, GLbyte, GLbyte> Binding::Color3b("glColor3b");
Function<void, const GLbyte*> Binding::Color3bv("glColor3bv");
Function<void, GLdouble, GLdouble, GLdouble> Binding::Color3d("glColor3d");
Function<void, const GLdouble*> Binding::Color3dv("glColor3dv");
Function<void, GLfloat, GLfloat, GLfloat> Binding::Color3f("glColor3f");
Function<void, GLfloat, GLfloat, GLfloat, GLfloat, GLfloat, GLfloat> Binding::Color3fVertex3fSUN(
    "glColor3fVertex3fSUN");
Function<void, const GLfloat*, const GLfloat*> Binding::Color3fVertex3fvSUN("glColor3fVertex3fvSUN");
Function<void, const GLfloat*> Binding::Color3fv("glColor3fv");
Function<void, GLhalfNV, GLhalfNV, GLhalfNV> Binding::Color3hNV("glColor3hNV");
Function<void, const GLhalfNV*> Binding::Color3hvNV("glColor3hvNV");
Function<void, GLint, GLint, GLint> Binding::Color3i("glColor3i");
Function<void, const GLint*> Binding::Color3iv("glColor3iv");
Function<void, GLshort, GLshort, GLshort> Binding::Color3s("glColor3s");
Function<void, const GLshort*> Binding::Color3sv("glColor3sv");
Function<void, GLubyte, GLubyte, GLubyte> Binding::Color3ub("glColor3ub");
Function<void, const GLubyte*> Binding::Color3ubv("glColor3ubv");
Function<void, GLuint, GLuint, GLuint> Binding::Color3ui("glColor3ui");
Function<void, const GLuint*> Binding::Color3uiv("glColor3uiv");
Function<void, GLushort, GLushort, GLushort> Binding::Color3us("glColor3us");
Function<void, const GLushort*> Binding::Color3usv("glColor3usv");
Function<void, GLfixed, GLfixed, GLfixed> Binding::Color3xOES("glColor3xOES");
Function<void, const GLfixed*> Binding::Color3xvOES("glColor3xvOES");
Function<void, GLbyte, GLbyte, GLbyte, GLbyte> Binding::Color4b("glColor4b");
Function<void, const GLbyte*> Binding::Color4bv("glColor4bv");
Function<void, GLdouble, GLdouble, GLdouble, GLdouble> Binding::Color4d("glColor4d");
Function<void, const GLdouble*> Binding::Color4dv("glColor4dv");
Function<void, GLfloat, GLfloat, GLfloat, GLfloat> Binding::Color4f("glColor4f");
Function<void, GLfloat, GLfloat, GLfloat, GLfloat, GLfloat, GLfloat, GLfloat, GLfloat, GLfloat, GLfloat>
    Binding::Color4fNormal3fVertex3fSUN("glColor4fNormal3fVertex3fSUN");
Function<void, const GLfloat*, const GLfloat*, const GLfloat*> Binding::Color4fNormal3fVertex3fvSUN(
    "glColor4fNormal3fVertex3fvSUN");
Function<void, const GLfloat*> Binding::Color4fv("glColor4fv");
Function<void, GLhalfNV, GLhalfNV, GLhalfNV, GLhalfNV> Binding::Color4hNV("glColor4hNV");
Function<void, const GLhalfNV*> Binding::Color4hvNV("glColor4hvNV");
Function<void, GLint, GLint, GLint, GLint> Binding::Color4i("glColor4i");
Function<void, const GLint*> Binding::Color4iv("glColor4iv");
Function<void, GLshort, GLshort, GLshort, GLshort> Binding::Color4s("glColor4s");
Function<void, const GLshort*> Binding::Color4sv("glColor4sv");
Function<void, GLubyte, GLubyte, GLubyte, GLubyte> Binding::Color4ub("glColor4ub");
Function<void, GLubyte, GLubyte, GLubyte, GLubyte, GLfloat, GLfloat> Binding::Color4ubVertex2fSUN(
    "glColor4ubVertex2fSUN");
Function<void, const GLubyte*, const GLfloat*> Binding::Color4ubVertex2fvSUN("glColor4ubVertex2fvSUN");
Function<void, GLubyte, GLubyte, GLubyte, GLubyte, GLfloat, GLfloat, GLfloat> Binding::Color4ubVertex3fSUN(
    "glColor4ubVertex3fSUN");
Function<void, const GLubyte*, const GLfloat*> Binding::Color4ubVertex3fvSUN("glColor4ubVertex3fvSUN");
Function<void, const GLubyte*> Binding::Color4ubv("glColor4ubv");
Function<void, GLuint, GLuint, GLuint, GLuint> Binding::Color4ui("glColor4ui");
Function<void, const GLuint*> Binding::Color4uiv("glColor4uiv");
Function<void, GLushort, GLushort, GLushort, GLushort> Binding::Color4us("glColor4us");
Function<void, const GLushort*> Binding::Color4usv("glColor4usv");
Function<void, GLfixed, GLfixed, GLfixed, GLfixed> Binding::Color4xOES("glColor4xOES");
Function<void, const GLfixed*> Binding::Color4xvOES("glColor4xvOES");
Function<void, GLint, GLenum, GLsizei> Binding::ColorFormatNV("glColorFormatNV");
Function<void, GLenum, GLuint, GLuint, GLuint, GLuint, GLuint, GLuint> Binding::ColorFragmentOp1ATI(
    "glColorFragmentOp1ATI");
Function<void, GLenum, GLuint, GLuint, GLuint, GLuint, GLuint, GLuint, GLuint, GLuint, GLuint>
    Binding::ColorFragmentOp2ATI("glColorFragmentOp2ATI");
Function<void, GLenum, GLuint, GLuint, GLuint, GLuint, GLuint, GLuint, GLuint, GLuint, GLuint, GLuint, GLuint, GLuint>
    Binding::ColorFragmentOp3ATI("glColorFragmentOp3ATI");
Function<void, GLboolean, GLboolean, GLboolean, GLboolean> Binding::ColorMask("glColorMask");
Function<void, GLuint, GLboolean, GLboolean, GLboolean, GLboolean> Binding::ColorMaskIndexedEXT(
    "glColorMaskIndexedEXT");
Function<void, GLuint, GLboolean, GLboolean, GLboolean, GLboolean> Binding::ColorMaski("glColorMaski");
Function<void, GLenum, GLenum> Binding::ColorMaterial("glColorMaterial");
Function<void, GLenum, GLuint> Binding::ColorP3ui("glColorP3ui");
Function<void, GLenum, const GLuint*> Binding::ColorP3uiv("glColorP3uiv");
Function<void, GLenum, GLuint> Binding::ColorP4ui("glColorP4ui");
Function<void, GLenum, const GLuint*> Binding::ColorP4uiv("glColorP4uiv");
Function<void, GLint, GLenum, GLsizei, const void*> Binding::ColorPointer("glColorPointer");
Function<void, GLint, GLenum, GLsizei, GLsizei, const void*> Binding::ColorPointerEXT("glColorPointerEXT");
Function<void, GLint, GLenum, GLint, const void**, GLint> Binding::ColorPointerListIBM("glColorPointerListIBM");
Function<void, GLint, GLenum, const void**> Binding::ColorPointervINTEL("glColorPointervINTEL");
Function<void, GLenum, GLsizei, GLsizei, GLenum, GLenum, const void*> Binding::ColorSubTable("glColorSubTable");
Function<void, GLenum, GLsizei, GLsizei, GLenum, GLenum, const void*> Binding::ColorSubTableEXT("glColorSubTableEXT");
Function<void, GLenum, GLenum, GLsizei, GLenum, GLenum, const void*> Binding::ColorTable("glColorTable");
Function<void, GLenum, GLenum, GLsizei, GLenum, GLenum, const void*> Binding::ColorTableEXT("glColorTableEXT");
Function<void, GLenum, GLenum, const GLfloat*> Binding::ColorTableParameterfv("glColorTableParameterfv");
Function<void, GLenum, GLenum, const GLfloat*> Binding::ColorTableParameterfvSGI("glColorTableParameterfvSGI");
Function<void, GLenum, GLenum, const GLint*> Binding::ColorTableParameteriv("glColorTableParameteriv");
Function<void, GLenum, GLenum, const GLint*> Binding::ColorTableParameterivSGI("glColorTableParameterivSGI");
Function<void, GLenum, GLenum, GLsizei, GLenum, GLenum, const void*> Binding::ColorTableSGI("glColorTableSGI");
Function<void, GLenum, GLenum, GLenum, GLenum, GLenum, GLenum> Binding::CombinerInputNV("glCombinerInputNV");
Function<void, GLenum, GLenum, GLenum, GLenum, GLenum, GLenum, GLenum, GLboolean, GLboolean, GLboolean>
    Binding::CombinerOutputNV("glCombinerOutputNV");
Function<void, GLenum, GLfloat> Binding::CombinerParameterfNV("glCombinerParameterfNV");
Function<void, GLenum, const GLfloat*> Binding::CombinerParameterfvNV("glCombinerParameterfvNV");
Function<void, GLenum, GLint> Binding::CombinerParameteriNV("glCombinerParameteriNV");
Function<void, GLenum, const GLint*> Binding::CombinerParameterivNV("glCombinerParameterivNV");
Function<void, GLenum, GLenum, const GLfloat*> Binding::CombinerStageParameterfvNV("glCombinerStageParameterfvNV");
Function<void, GLuint, GLuint> Binding::CommandListSegmentsNV("glCommandListSegmentsNV");
Function<void, GLuint> Binding::CompileCommandListNV("glCompileCommandListNV");
Function<void, GLuint> Binding::CompileShader("glCompileShader");
Function<void, GLhandleARB> Binding::CompileShaderARB("glCompileShaderARB");
Function<void, GLuint, GLsizei, const GLchar* const*, const GLint*> Binding::CompileShaderIncludeARB(
    "glCompileShaderIncludeARB");
Function<void, GLenum, GLenum, GLint, GLenum, GLsizei, GLint, GLsizei, const void*>
    Binding::CompressedMultiTexImage1DEXT("glCompressedMultiTexImage1DEXT");
Function<void, GLenum, GLenum, GLint, GLenum, GLsizei, GLsizei, GLint, GLsizei, const void*>
    Binding::CompressedMultiTexImage2DEXT("glCompressedMultiTexImage2DEXT");
Function<void, GLenum, GLenum, GLint, GLenum, GLsizei, GLsizei, GLsizei, GLint, GLsizei, const void*>
    Binding::CompressedMultiTexImage3DEXT("glCompressedMultiTexImage3DEXT");
Function<void, GLenum, GLenum, GLint, GLint, GLsizei, GLenum, GLsizei, const void*>
    Binding::CompressedMultiTexSubImage1DEXT("glCompressedMultiTexSubImage1DEXT");
Function<void, GLenum, GLenum, GLint, GLint, GLint, GLsizei, GLsizei, GLenum, GLsizei, const void*>
    Binding::CompressedMultiTexSubImage2DEXT("glCompressedMultiTexSubImage2DEXT");
Function<void, GLenum, GLenum, GLint, GLint, GLint, GLint, GLsizei, GLsizei, GLsizei, GLenum, GLsizei, const void*>
    Binding::CompressedMultiTexSubImage3DEXT("glCompressedMultiTexSubImage3DEXT");
Function<void, GLenum, GLint, GLenum, GLsizei, GLint, GLsizei, const void*> Binding::CompressedTexImage1D(
    "glCompressedTexImage1D");
Function<void, GLenum, GLint, GLenum, GLsizei, GLint, GLsizei, const void*> Binding::CompressedTexImage1DARB(
    "glCompressedTexImage1DARB");
Function<void, GLenum, GLint, GLenum, GLsizei, GLsizei, GLint, GLsizei, const void*> Binding::CompressedTexImage2D(
    "glCompressedTexImage2D");
Function<void, GLenum, GLint, GLenum, GLsizei, GLsizei, GLint, GLsizei, const void*> Binding::CompressedTexImage2DARB(
    "glCompressedTexImage2DARB");
Function<void, GLenum, GLint, GLenum, GLsizei, GLsizei, GLsizei, GLint, GLsizei, const void*>
    Binding::CompressedTexImage3D("glCompressedTexImage3D");
Function<void, GLenum, GLint, GLenum, GLsizei, GLsizei, GLsizei, GLint, GLsizei, const void*>
    Binding::CompressedTexImage3DARB("glCompressedTexImage3DARB");
Function<void, GLenum, GLint, GLint, GLsizei, GLenum, GLsizei, const void*> Binding::CompressedTexSubImage1D(
    "glCompressedTexSubImage1D");
Function<void, GLenum, GLint, GLint, GLsizei, GLenum, GLsizei, const void*> Binding::CompressedTexSubImage1DARB(
    "glCompressedTexSubImage1DARB");
Function<void, GLenum, GLint, GLint, GLint, GLsizei, GLsizei, GLenum, GLsizei, const void*>
    Binding::CompressedTexSubImage2D("glCompressedTexSubImage2D");
Function<void, GLenum, GLint, GLint, GLint, GLsizei, GLsizei, GLenum, GLsizei, const void*>
    Binding::CompressedTexSubImage2DARB("glCompressedTexSubImage2DARB");
Function<void, GLenum, GLint, GLint, GLint, GLint, GLsizei, GLsizei, GLsizei, GLenum, GLsizei, const void*>
    Binding::CompressedTexSubImage3D("glCompressedTexSubImage3D");
Function<void, GLenum, GLint, GLint, GLint, GLint, GLsizei, GLsizei, GLsizei, GLenum, GLsizei, const void*>
    Binding::CompressedTexSubImage3DARB("glCompressedTexSubImage3DARB");
Function<void, GLuint, GLenum, GLint, GLenum, GLsizei, GLint, GLsizei, const void*>
    Binding::CompressedTextureImage1DEXT("glCompressedTextureImage1DEXT");
Function<void, GLuint, GLenum, GLint, GLenum, GLsizei, GLsizei, GLint, GLsizei, const void*>
    Binding::CompressedTextureImage2DEXT("glCompressedTextureImage2DEXT");
Function<void, GLuint, GLenum, GLint, GLenum, GLsizei, GLsizei, GLsizei, GLint, GLsizei, const void*>
    Binding::CompressedTextureImage3DEXT("glCompressedTextureImage3DEXT");
Function<void, GLuint, GLint, GLint, GLsizei, GLenum, GLsizei, const void*> Binding::CompressedTextureSubImage1D(
    "glCompressedTextureSubImage1D");
Function<void, GLuint, GLenum, GLint, GLint, GLsizei, GLenum, GLsizei, const void*>
    Binding::CompressedTextureSubImage1DEXT("glCompressedTextureSubImage1DEXT");
Function<void, GLuint, GLint, GLint, GLint, GLsizei, GLsizei, GLenum, GLsizei, const void*>
    Binding::CompressedTextureSubImage2D("glCompressedTextureSubImage2D");
Function<void, GLuint, GLenum, GLint, GLint, GLint, GLsizei, GLsizei, GLenum, GLsizei, const void*>
    Binding::CompressedTextureSubImage2DEXT("glCompressedTextureSubImage2DEXT");
Function<void, GLuint, GLint, GLint, GLint, GLint, GLsizei, GLsizei, GLsizei, GLenum, GLsizei, const void*>
    Binding::CompressedTextureSubImage3D("glCompressedTextureSubImage3D");
Function<void, GLuint, GLenum, GLint, GLint, GLint, GLint, GLsizei, GLsizei, GLsizei, GLenum, GLsizei, const void*>
    Binding::CompressedTextureSubImage3DEXT("glCompressedTextureSubImage3DEXT");
Function<void, GLenum, GLfloat> Binding::ConservativeRasterParameterfNV("glConservativeRasterParameterfNV");
Function<void, GLenum, GLint> Binding::ConservativeRasterParameteriNV("glConservativeRasterParameteriNV");
Function<void, GLenum, GLenum, GLsizei, GLenum, GLenum, const void*> Binding::ConvolutionFilter1D(
    "glConvolutionFilter1D");
Function<void, GLenum, GLenum, GLsizei, GLenum, GLenum, const void*> Binding::ConvolutionFilter1DEXT(
    "glConvolutionFilter1DEXT");
Function<void, GLenum, GLenum, GLsizei, GLsizei, GLenum, GLenum, const void*> Binding::ConvolutionFilter2D(
    "glConvolutionFilter2D");
Function<void, GLenum, GLenum, GLsizei, GLsizei, GLenum, GLenum, const void*> Binding::ConvolutionFilter2DEXT(
    "glConvolutionFilter2DEXT");
Function<void, GLenum, GLenum, GLfloat> Binding::ConvolutionParameterf("glConvolutionParameterf");
Function<void, GLenum, GLenum, GLfloat> Binding::ConvolutionParameterfEXT("glConvolutionParameterfEXT");
Function<void, GLenum, GLenum, const GLfloat*> Binding::ConvolutionParameterfv("glConvolutionParameterfv");
Function<void, GLenum, GLenum, const GLfloat*> Binding::ConvolutionParameterfvEXT("glConvolutionParameterfvEXT");
Function<void, GLenum, GLenum, GLint> Binding::ConvolutionParameteri("glConvolutionParameteri");
Function<void, GLenum, GLenum, GLint> Binding::ConvolutionParameteriEXT("glConvolutionParameteriEXT");
Function<void, GLenum, GLenum, const GLint*> Binding::ConvolutionParameteriv("glConvolutionParameteriv");
Function<void, GLenum, GLenum, const GLint*> Binding::ConvolutionParameterivEXT("glConvolutionParameterivEXT");
Function<void, GLenum, GLenum, GLfixed> Binding::ConvolutionParameterxOES("glConvolutionParameterxOES");
Function<void, GLenum, GLenum, const GLfixed*> Binding::ConvolutionParameterxvOES("glConvolutionParameterxvOES");
Function<void, GLenum, GLenum, GLintptr, GLintptr, GLsizeiptr> Binding::CopyBufferSubData("glCopyBufferSubData");
Function<void, GLenum, GLsizei, GLint, GLint, GLsizei> Binding::CopyColorSubTable("glCopyColorSubTable");
Function<void, GLenum, GLsizei, GLint, GLint, GLsizei> Binding::CopyColorSubTableEXT("glCopyColorSubTableEXT");
Function<void, GLenum, GLenum, GLint, GLint, GLsizei> Binding::CopyColorTable("glCopyColorTable");
Function<void, GLenum, GLenum, GLint, GLint, GLsizei> Binding::CopyColorTableSGI("glCopyColorTableSGI");
Function<void, GLenum, GLenum, GLint, GLint, GLsizei> Binding::CopyConvolutionFilter1D("glCopyConvolutionFilter1D");
Function<void, GLenum, GLenum, GLint, GLint, GLsizei> Binding::CopyConvolutionFilter1DEXT(
    "glCopyConvolutionFilter1DEXT");
Function<void, GLenum, GLenum, GLint, GLint, GLsizei, GLsizei> Binding::CopyConvolutionFilter2D(
    "glCopyConvolutionFilter2D");
Function<void, GLenum, GLenum, GLint, GLint, GLsizei, GLsizei> Binding::CopyConvolutionFilter2DEXT(
    "glCopyConvolutionFilter2DEXT");
Function<void, GLuint, GLenum, GLint, GLint, GLint, GLint, GLuint, GLenum, GLint, GLint, GLint, GLint, GLsizei, GLsizei,
         GLsizei>
    Binding::CopyImageSubData("glCopyImageSubData");
Function<void, GLuint, GLenum, GLint, GLint, GLint, GLint, GLuint, GLenum, GLint, GLint, GLint, GLint, GLsizei, GLsizei,
         GLsizei>
    Binding::CopyImageSubDataNV("glCopyImageSubDataNV");
Function<void, GLenum, GLenum, GLint, GLenum, GLint, GLint, GLsizei, GLint> Binding::CopyMultiTexImage1DEXT(
    "glCopyMultiTexImage1DEXT");
Function<void, GLenum, GLenum, GLint, GLenum, GLint, GLint, GLsizei, GLsizei, GLint> Binding::CopyMultiTexImage2DEXT(
    "glCopyMultiTexImage2DEXT");
Function<void, GLenum, GLenum, GLint, GLint, GLint, GLint, GLsizei> Binding::CopyMultiTexSubImage1DEXT(
    "glCopyMultiTexSubImage1DEXT");
Function<void, GLenum, GLenum, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei> Binding::CopyMultiTexSubImage2DEXT(
    "glCopyMultiTexSubImage2DEXT");
Function<void, GLenum, GLenum, GLint, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei>
    Binding::CopyMultiTexSubImage3DEXT("glCopyMultiTexSubImage3DEXT");
Function<void, GLuint, GLuint, GLintptr, GLintptr, GLsizeiptr> Binding::CopyNamedBufferSubData(
    "glCopyNamedBufferSubData");
Function<void, GLuint, GLuint> Binding::CopyPathNV("glCopyPathNV");
Function<void, GLint, GLint, GLsizei, GLsizei, GLenum> Binding::CopyPixels("glCopyPixels");
Function<void, GLenum, GLint, GLenum, GLint, GLint, GLsizei, GLint> Binding::CopyTexImage1D("glCopyTexImage1D");
Function<void, GLenum, GLint, GLenum, GLint, GLint, GLsizei, GLint> Binding::CopyTexImage1DEXT("glCopyTexImage1DEXT");
Function<void, GLenum, GLint, GLenum, GLint, GLint, GLsizei, GLsizei, GLint> Binding::CopyTexImage2D(
    "glCopyTexImage2D");
Function<void, GLenum, GLint, GLenum, GLint, GLint, GLsizei, GLsizei, GLint> Binding::CopyTexImage2DEXT(
    "glCopyTexImage2DEXT");
Function<void, GLenum, GLint, GLint, GLint, GLint, GLsizei> Binding::CopyTexSubImage1D("glCopyTexSubImage1D");
Function<void, GLenum, GLint, GLint, GLint, GLint, GLsizei> Binding::CopyTexSubImage1DEXT("glCopyTexSubImage1DEXT");
Function<void, GLenum, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei> Binding::CopyTexSubImage2D(
    "glCopyTexSubImage2D");
Function<void, GLenum, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei> Binding::CopyTexSubImage2DEXT(
    "glCopyTexSubImage2DEXT");
Function<void, GLenum, GLint, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei> Binding::CopyTexSubImage3D(
    "glCopyTexSubImage3D");
Function<void, GLenum, GLint, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei> Binding::CopyTexSubImage3DEXT(
    "glCopyTexSubImage3DEXT");
Function<void, GLuint, GLenum, GLint, GLenum, GLint, GLint, GLsizei, GLint> Binding::CopyTextureImage1DEXT(
    "glCopyTextureImage1DEXT");
Function<void, GLuint, GLenum, GLint, GLenum, GLint, GLint, GLsizei, GLsizei, GLint> Binding::CopyTextureImage2DEXT(
    "glCopyTextureImage2DEXT");
Function<void, GLuint, GLint, GLint, GLint, GLint, GLsizei> Binding::CopyTextureSubImage1D("glCopyTextureSubImage1D");
Function<void, GLuint, GLenum, GLint, GLint, GLint, GLint, GLsizei> Binding::CopyTextureSubImage1DEXT(
    "glCopyTextureSubImage1DEXT");
Function<void, GLuint, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei> Binding::CopyTextureSubImage2D(
    "glCopyTextureSubImage2D");
Function<void, GLuint, GLenum, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei> Binding::CopyTextureSubImage2DEXT(
    "glCopyTextureSubImage2DEXT");
Function<void, GLuint, GLint, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei> Binding::CopyTextureSubImage3D(
    "glCopyTextureSubImage3D");
Function<void, GLuint, GLenum, GLint, GLint, GLint, GLint, GLint, GLint, GLsizei, GLsizei>
    Binding::CopyTextureSubImage3DEXT("glCopyTextureSubImage3DEXT");
Function<void, GLsizei, GLenum, const void*, GLuint, GLenum, GLenum, const GLfloat*> Binding::CoverFillPathInstancedNV(
    "glCoverFillPathInstancedNV");
Function<void, GLuint, GLenum> Binding::CoverFillPathNV("glCoverFillPathNV");
Function<void, GLsizei, GLenum, const void*, GLuint, GLenum, GLenum, const GLfloat*>
    Binding::CoverStrokePathInstancedNV("glCoverStrokePathInstancedNV");
Function<void, GLuint, GLenum> Binding::CoverStrokePathNV("glCoverStrokePathNV");
Function<void, GLenum> Binding::CoverageModulationNV("glCoverageModulationNV");
Function<void, GLsizei, const GLfloat*> Binding::CoverageModulationTableNV("glCoverageModulationTableNV");
Function<void, GLsizei, GLuint*> Binding::CreateBuffers("glCreateBuffers");
Function<void, GLsizei, GLuint*> Binding::CreateCommandListsNV("glCreateCommandListsNV");
Function<void, GLsizei, GLuint*> Binding::CreateFramebuffers("glCreateFramebuffers");
Function<void, GLsizei, GLuint*> Binding::CreateMemoryObjectsEXT("glCreateMemoryObjectsEXT");
Function<void, GLuint, GLuint*> Binding::CreatePerfQueryINTEL("glCreatePerfQueryINTEL");
Function<GLuint> Binding::CreateProgram("glCreateProgram");
Function<GLhandleARB> Binding::CreateProgramObjectARB("glCreateProgramObjectARB");
Function<void, GLsizei, GLuint*> Binding::CreateProgramPipelines("glCreateProgramPipelines");
Function<void, GLenum, GLsizei, GLuint*> Binding::CreateQueries("glCreateQueries");
Function<void, GLsizei, GLuint*> Binding::CreateRenderbuffers("glCreateRenderbuffers");
Function<void, GLsizei, GLuint*> Binding::CreateSamplers("glCreateSamplers");
Function<GLuint, GLenum> Binding::CreateShader("glCreateShader");
Function<GLhandleARB, GLenum> Binding::CreateShaderObjectARB("glCreateShaderObjectARB");
Function<GLuint, GLenum, const GLchar*> Binding::CreateShaderProgramEXT("glCreateShaderProgramEXT");
Function<GLuint, GLenum, GLsizei, const GLchar* const*> Binding::CreateShaderProgramv("glCreateShaderProgramv");
Function<void, GLsizei, GLuint*> Binding::CreateStatesNV("glCreateStatesNV");
Function<GLsync, _cl_context*, _cl_event*, UnusedMask> Binding::CreateSyncFromCLeventARB("glCreateSyncFromCLeventARB");
Function<void, GLenum, GLsizei, GLuint*> Binding::CreateTextures("glCreateTextures");
Function<void, GLsizei, GLuint*> Binding::CreateTransformFeedbacks("glCreateTransformFeedbacks");
Function<void, GLsizei, GLuint*> Binding::CreateVertexArrays("glCreateVertexArrays");
Function<void, GLenum> Binding::CullFace("glCullFace");
Function<void, GLenum, GLdouble*> Binding::CullParameterdvEXT("glCullParameterdvEXT");
Function<void, GLenum, GLfloat*> Binding::CullParameterfvEXT("glCullParameterfvEXT");
Function<void, GLint> Binding::CurrentPaletteMatrixARB("glCurrentPaletteMatrixARB");



}  // namespace glbinding
