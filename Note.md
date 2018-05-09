# Ork Renderer Note
## Questions
- why multiply -1
```
void
Model::LoadModel(const std::string & file_path)
{
  scene = aiImportFile(file_path.c_str(), aiProcessPreset_TargetRealtime_Quality);
  for (unsigned int i=0; i<scene->mNumMeshes; i++){
    for (unsigned int j=0; j<scene->mMeshes[i]->mNumVertices; j++){
      scene->mMeshes[i]->mVertices[j].x *= -1;
      scene->mMeshes[i]->mVertices[j].y *= -1;
      scene->mMeshes[i]->mVertices[j].z *= -1;
    }
  }

  recursiveTextureLoad(scene, scene->mRootNode);
}
```

## Assimp usage

## OpenGl Usage
### Glulookat
若不设置，默认相机在世界坐标系原点，y轴向上，朝向世界坐标系z轴的负方向！gllookat一定要放在模型变换的前面，否则会显示不明。

### 复习矩阵的左乘右乘变换
#### 1. 矩阵变换的解释
C=BA，对B来说，是左乘，相当于对B做行变换；对A来说，是右乘，相当于对A做列变换。

#### 2. 齐次变换的解释
对于变换矩阵R = Rz * Ry * Rx，对某个A作用，A‘=RA=Rz * Ry * Rx×A

__可以有两种解释：__
- 相对于参考坐标系O（固定）的变换。先绕x轴旋转，再绕y轴旋转，最后z轴
- 相对于当前坐标系的变换。先绕z轴旋转，再绕新坐标系的y轴旋转，最后绕新坐标系的x轴旋转。

### Opengl坐标变换
#### 作用顺序
OpenGL对模型进行旋转、平移和缩放。用到三个子函数： glTranslate*(x, y, z) 、 glRotate*(x, y, z) 、 glScale*(x, y, z) 。每个函数都会产生一个矩阵，并 __右乘当前矩__
对与变换：

    glRotatef(45.0, 0.0, 0.0, 1.0);
	glTranslatef(3.0, 0.0, 0.0);

这 两个变换，可以看成：

    glMultMatrixf(R);
	glMultMatrixf(T);

R,T 都是右乘到CTM（当前变换矩阵）：CTM = CTM * R * T
#### 视图流

__MODELVIEW MATRIX__

模型视图矩阵，描述从模型的局部坐标系到eye坐标系的变换。即，该矩阵左乘模型上点的局部坐标，得到模型上的点在eye坐标系下的坐标。该矩阵也表示，模型局部坐标系在eye坐标系下的pose。

__OpenGL中default view的朝向问题__

面向屏幕，eye coord的x方向向右，y方向向上，z方向从里向外。但是，观察到只有当model的z component为负时，才能在视场中看到物体。和机器视觉中camera coordinate的情况相反。
即只有当 modelview matrix的translation的z分量为负时，才能在视野中看到物体。

__OpenGl 模拟pinhole camera问题__

解决上一点的问题需要在modelview matrix上左乘一个旋转矩阵Rotx(pi)，来达到把opengl中的eye coord和经典的pinhole camera 的coord重合的目的。

__读取buffer图像，保存到opencv矩阵问题__

ork中使用如下代码：
```
glReadBuffer(GL_COLOR_ATTACHMENT0);
glReadPixels(0, 0, renderer_->width_, renderer_->height_, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());
```
需要注意的点是，opengl和opencv图像的原点不一样，opengl的原点在左下角，而opencv在左上角！因此，拷贝到image中的图像如果用cv显示出来是上下颠倒的，需要flip操作一次！

拷贝buffer中的图像，在CSDN上看到的代码
```
// in this way , all buttons will not been saved !
// OpenGl2Bmp
bool ScreenShot(const char* filename)
{
    GLenum lastBuffer;
    GLbyte* pBits = 0; // data
    unsigned long lImageSize;
    GLint iViewport[4]; // view
 
    glGetIntegerv(GL_VIEWPORT, iViewport);
    lImageSize = iViewport[2] * iViewport[3] * 3;// w*h*3
 
    pBits = (GLbyte*)new unsigned char[lImageSize];
    if (!pBits) return false;
 
// 从color buffer中读取数据
    glPixelStorei(GL_PACK_ALIGNMENT, 1);// using alginment of 1 byte
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);// >0, GL_PACK_ROW_LENGTH defines the number of pixels in a row.
    glPixelStorei(GL_PACK_SKIP_ROWS, 0);
    glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
//
    glGetIntegerv(GL_READ_BUFFER, (GLint*)&lastBuffer);
// afxDump << lastBuffer << "\n";// 0x405 =#define GL_BACK 
    glReadBuffer(GL_FRONT);// chang to
// bmp format
//afxDump << iViewport[2] << "\n";// 720 
//afxDump << iViewport[3] << "\n";  // 450 
    glReadPixels(iViewport[0],iViewport[1], iViewport[2], iViewport[3],
                GL_BGR_EXT, GL_UNSIGNED_BYTE, pBits);
//
    glReadBuffer(lastBuffer);// set back
//
    if (writeBMP(filename,(unsigned char*)pBits,iViewport[2],iViewport[3])) return true;
//
    return false;
}
```

## Develop log

### 2018-2-23
- 修改了Renderer3D的setModelRt 函数，使得输入为标准的opencv camera 坐标系表示法，并在roject_render_test中进行了测试
- 创建 view_generate.cpp 测试renderer_iterator，并和setModelRt进行相互验证
- 在renderer_iterator中加入了新的函数Rt_obj，按照标准opencv 相机坐标体系返回物体的pose，其原先自带的obj_R表示法好像有问题。使用obj_Rt后，render之后的图像是上下颠倒的（底层机理很简单）需要上下flip得到真实的图像。
- 注释掉了Renderer3D成员函数lookat里面的一段（原先用来center the model）

    
### 2018-2-24
- 在使用omesa库进行渲染时，出现了诡异的面上漏洞问题，在改用glut库之后问题消失了。
- renderer_iterator 的设置angle_min 和 angle_max设置缺乏设置函数，改变这两个变量理应改变angle_变量

    
   

