/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014- Centrum Wiskunde en Informatica
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
#ifndef COMPRESSION_EVAL_H
#define COMPRESSION_EVAL_H

#include <pcl/visualization/boost.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/cloud_codec_v2/point_cloud_codec_v2.h>
#include <pcl/cloud_codec_v2/impl/point_cloud_codec_v2_impl.hpp>

namespace pcl{

  namespace io{

	/**!
		\brief helper function to generate octree cloud codecs based 
		on the number of bits in the base and enhancement layer
	*/
    template<typename PointT> boost::shared_ptr<OctreePointCloudCodecV2<PointT> >
	  generatePCLOctreeCodecV2(int nr_bits_base_layer, int nr_bits_enh_layer, int nr_bits_colors, int i_frame_rate = 0, int color_coding_type = 0, bool do_centroid_coding = true);
	}

}

// --------------------------------------------------------------------------
//
// Copyright(C) 2007-2013
// Tamy Boubekeur
//                                                                            
// All rights reserved.                                                                                                        
//                                                                          
// --------------------------------------------------------------------------

#pragma once

#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <string>

// --------------------------------------------------------------------------
//
// Copyright(C) 2007-2013
// Tamy Boubekeur
//
// All rights reserved.
//
// --------------------------------------------------------------------------

#pragma once

#include <cmath>
#include <iostream>


// dalexiad (CERTH) additions
// ------------------------------------
#define _USE_MATH_DEFINES
#include <math.h>
// ------------------------------------


namespace Nano3D {

template<typename T> class Vec3D;

template <class T> bool operator!= (const Vec3D<T> & p1, const Vec3D<T> & p2) {
    return (p1[0] != p2[0] || p1[1] != p2[1] || p1[2] != p2[2]);
}

template <class T> const Vec3D<T> operator* (const Vec3D<T> & p, float factor) {
    return Vec3D<T> (p[0] * factor, p[1] * factor, p[2] * factor);
}

template <class T> const Vec3D<T> operator* (float factor, const Vec3D<T> & p) {
    return Vec3D<T> (p[0] * factor, p[1] * factor, p[2] * factor);
}

template <class T> const Vec3D<T> operator* (const Vec3D<T> & p1, const Vec3D<T> & p2) {
    return Vec3D<T> (p1[0] * p2[0], p1[1] * p2[1], p1[2] * p2[2]);
}

template <class T> const Vec3D<T> operator+ (const Vec3D<T> & p1, const Vec3D<T> & p2) {
    return Vec3D<T> (p1[0] + p2[0], p1[1] + p2[1], p1[2] + p2[2]);
}

template <class T> const Vec3D<T> operator- (const Vec3D<T> & p1, const Vec3D<T> & p2) {
    return Vec3D<T> (p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]);
}

template <class T> const Vec3D<T> operator- (const Vec3D<T> & p) {
    return Vec3D<T> (-p[0], -p[1], -p[2]);
}

template <class T> const Vec3D<T> operator/ (const Vec3D<T> & p, float divisor) {
    return Vec3D<T> (p[0]/divisor, p[1]/divisor, p[2]/divisor);
}

template <class T> bool operator== (const Vec3D<T> & p1, const Vec3D<T> & p2) {
    return (p1[0] == p2[0] && p1[1] == p2[1] && p1[2] == p2[2]);
}

template <class T> bool operator< (const Vec3D<T> & a, const Vec3D<T> & b) {
    return (a[0] < b[0] && a[1] < b[1] && a[2] < b[2]);
}

template <class T> bool operator>= (const Vec3D<T> & a, const Vec3D<T> & b) {
    return (a[0] >= b[0] || a[1] >= b[1] || a[2] >= b[2]);
}

/// Vector in 3 dimensions, with basics operators overloaded, templated by the precision.
template <typename T>
class Vec3D {
public:
    inline Vec3D (void)	{
        p[0] = p[1] = p[2] = T ();
    }
    inline Vec3D (T p0, T p1, T p2) {
        p[0] = p0;
        p[1] = p1;
        p[2] = p2;
    };
    inline Vec3D (const Vec3D & v) {
        init (v[0], v[1], v[2]);
    }
    inline Vec3D (T* pp) {
        p[0] = pp[0];
        p[1] = pp[1];
        p[2] = pp[2];
    };
    // ---------
    // Operators
    // ---------
    inline T& operator[] (int Index) {
        return (p[Index]);
    };
    inline const T& operator[] (int Index) const {
        return (p[Index]);
    };
    inline Vec3D& operator= (const Vec3D & P) {
        p[0] = P[0];
        p[1] = P[1];
        p[2] = P[2];
        return (*this);
    };
    inline Vec3D& operator+= (const Vec3D & P) {
        p[0] += P[0];
        p[1] += P[1];
        p[2] += P[2];
        return (*this);
    };
    inline Vec3D& operator-= (const Vec3D & P) {
        p[0] -= P[0];
        p[1] -= P[1];
        p[2] -= P[2];
        return (*this);
    };
    inline Vec3D& operator*= (const Vec3D & P) {
        p[0] *= P[0];
        p[1] *= P[1];
        p[2] *= P[2];
        return (*this);
    };
    inline Vec3D& operator*= (T s) {
        p[0] *= s;
        p[1] *= s;
        p[2] *= s;
        return (*this);
    };
    inline Vec3D& operator/= (const Vec3D & P) {
        p[0] /= P[0];
        p[1] /= P[1];
        p[2] /= P[2];
        return (*this);
    };
    inline Vec3D& operator/= (T s) {
        p[0] /= s;
        p[1] /= s;
        p[2] /= s;
        return (*this);
    };

    //---------------------------------------------------------------

    inline Vec3D & init (T x, T y, T z) {
        p[0] = x;
        p[1] = y;
        p[2] = z;
        return (*this);
    };
    inline T getSquaredLength() const {
        return (dotProduct (*this, *this));
    };
    inline T getLength() const {
        return (T)sqrt (getSquaredLength());
    };
    /// Return length after normalization
    inline T normalize (void) {
        T length = getLength();
        if (length == 0.0f)
            return 0;
        T rezLength = 1.0f / length;
        p[0] *= rezLength;
        p[1] *= rezLength;
        p[2] *= rezLength;
        return length;
    };
    inline void fromTo (const Vec3D & P1, const Vec3D & P2) {
        p[0] = P2[0] - P1[0];
        p[1] = P2[1] - P1[1];
        p[2] = P2[2] - P1[2];
    };
    inline float transProduct (const Vec3D & v) const {
        return (p[0]*v[0] + p[1]*v[1] + p[2]*v[2]);
    }
    inline void getTwoOrthogonals (Vec3D & u, Vec3D & v) const {
        if (fabs(p[0]) < fabs(p[1])) {
            if (fabs(p[0]) < fabs(p[2]))
                u = Vec3D (0, -p[2], p[1]);
            else
                u = Vec3D (-p[1], p[0], 0);
        } else {
            if (fabs(p[1]) < fabs(p[2]))
                u = Vec3D (p[2], 0, -p[0]);
            else
                u = Vec3D(-p[1], p[0], 0);
        }
        v = crossProduct (*this, u);
    }
    inline Vec3D projectOn (const Vec3D & N, const Vec3D & P) const {
        T w = dotProduct (((*this) - P), N);
        return (*this) - (N * w);
    }
    static inline Vec3D segment (const Vec3D & a, const Vec3D & b) {
        Vec3D r;
        r[0] = b[0] - a[0];
        r[1] = b[1] - a[1];
        r[2] = b[2] - a[2];
        return r;
    };
    static inline Vec3D crossProduct(const Vec3D & a, const Vec3D & b) {
        Vec3D result;
        result[0] = a[1] * b[2] - a[2] * b[1];
        result[1] = a[2] * b[0] - a[0] * b[2];
        result[2] = a[0] * b[1] - a[1] * b[0];
        return(result);
    }
    static inline T dotProduct(const Vec3D & a, const Vec3D & b) {
        return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
    }
    static inline T squaredDistance (const Vec3D &v1, const Vec3D &v2) {
        Vec3D tmp = v1 - v2;
        return (tmp.getSquaredLength());
    }
    static inline T distance (const Vec3D &v1, const Vec3D &v2) {
        Vec3D tmp = v1 - v2;
        return (tmp.getLength());
    }
    static inline Vec3D interpolate (const Vec3D & u, const Vec3D & v, T alpha) {
        return (u * (1.0f - alpha) + v * alpha);
    }

    // cartesion to polar coordinates
    // result:
    // [0] = length
    // [1] = angle with z-axis
    // [2] = angle of projection into x,y, plane with x-axis
    static inline Vec3D cartesianToPolar (const Vec3D &v) {
        Vec3D polar;
        polar[0] = v.getLength();
        if (v[2] > 0.0f)
            polar[1] = (T) atan (sqrt (v[0] * v[0] + v[1] * v[1]) / v[2]);
        else if (v[2] < 0.0f)
            polar[1] = (T) atan (sqrt (v[0] * v[0] + v[1] * v[1]) / v[2]) + M_PI;
        else
            polar[1] = M_PI * 0.5f;
        if (v[0] > 0.0f)
            polar[2] = (T) atan (v[1] / v[0]);
        else if (v[0] < 0.0f)
            polar[2] = (T) atan (v[1] / v[0]) + M_PI;
        else if (v[1] > 0)
            polar[2] = M_PI * 0.5f;
        else
            polar[2] = -M_PI * 0.5;
        return polar;
    }

    // polar to cartesion coordinates
    // input:
    // [0] = length
    // [1] = angle with z-axis
    // [2] = angle of projection into x,y, plane with x-axis
    static inline Vec3D polarToCartesian (const Vec3D & v) {
        Vec3D cart;
        cart[0] = v[0] * (T) sin (v[1]) * (T) cos (v[2]);
        cart[1] = v[0] * (T) sin (v[1]) * (T) sin (v[2]);
        cart[2] = v[0] * (T) cos (v[1]);
        return cart;
    }
    static inline Vec3D projectOntoVector (const Vec3D & v1, const Vec3D & v2) {
        return v2 * dotProduct (v1, v2);
    }
    inline Vec3D transformIn (const Vec3D & pos, const Vec3D & n, const Vec3D & u, const Vec3D & v) const {
        Vec3D q = (*this) - pos;
        return Vec3D (u[0]*q[0] + u[1]*q[1] + u[2]*q[2],
                      v[0]*q[0] + v[1]*q[1] + v[2]*q[2],
                      n[0]*q[0] + n[1]*q[1] + n[2]*q[2]);
    }

protected:
    T p[3];
};

template <class T> inline Vec3D<T> swap (Vec3D<T> & P, Vec3D<T> & Q) {
    Vec3D<T> tmp = P;
    P = Q;
    Q = tmp;
}

template <class T> std::ostream & operator<< (std::ostream & output, const Vec3D<T> & v) {
    output << v[0] << " " << v[1] << " " << v[2];
    return output;
}

template <class T> std::istream & operator>> (std::istream & input, Vec3D<T> & v) {
    input >> v[0] >> v[1] >> v[2];
    return input;
}

typedef Vec3D<float> Vec3Df;
typedef Vec3D<double> Vec3Dd;
typedef Vec3D<int> Vec3Di;

}
// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:




// dalexiad (CERTH) additions
// ------------------------------------
#define MAX_NUM_OF_TEXTURE_IMAGES 5


//(Nano3D::Mesh is a class that is shared-used in both capturer and renderer)

#pragma once

//Declaration of some commonly used functions
void HConvertProjectiveToNormalized(const float u, const float v, 
	float* A, 
	float &Xn, float &Yn);
void HDistortNormalized(const float Xn, const float Yn, float* mD, float &Xd, float &Yd);
//Without radial distortion
void HConvertProjectiveToRealWorldStandard(float u, float v, float Z, float* A, float* XYZ);
//With radial distortion
void HConvertProjectiveToRealWorld(float u, float v, float Z, float* A, float* D, float* XYZ);
void HTransform3D(float* XYZIn, float* R, float* T, float* XYZOut);
void HTransform3DInverse(float* XYZIn, float* R, float* T, float* XYZOut);
//No radial distortion
void HConvertRealWorld3DToProjectiveStandard(const float *X,  float* A, float& u, float &v);
//With radial distortion
void HConvertRealWorld3DToProjective(const float *X,  float* A, float* D, float& u, float &v);




class CRGBDepthCameraParams
{
public:
	CRGBDepthCameraParams(void);

	//Extrinsic parameters
	float m_R[9];
	float m_T[3];

	//Intrinsic Depth params
	float m_ADepth[9];


	//Intrinsic RGB params
	float m_ARGB[9];
	float m_DRGB[5];


	float m_depthToRGB_R[9];
	float m_depthToRGB_T[3];

	void SetEYEExtrinsics();
	void SetAllParameters(const float* R, const float* T, const float* ADepth, const float* ARGB, const float* DRGB, const float* depthToRGB_R, const float* depthToRGB_T, 
		bool hasRGBDistortion, bool hasRGBDepth_RT);

	bool m_hasRGBDistortion;
	bool m_hasRGBDepth_RT;

	//Copy-Assignment operator
	CRGBDepthCameraParams & operator= (const CRGBDepthCameraParams & other)
	{
		if (this != &other) // protect against invalid self-assignment
		{
			SetAllParameters(other.m_R, other.m_T, other.m_ADepth, other.m_ARGB, other.m_DRGB, other.m_depthToRGB_R, other.m_depthToRGB_T, 
				other.m_hasRGBDistortion, other.m_hasRGBDepth_RT);

		}
		// by convention, always return *this
		return *this;
	}


	void ConvertProjectiveDepthToRealWorldRGB(float u, float v, float Z, float *X);

	void Transform3DWithExtrinsics(float* XYZIn, float* XYZOut);
	void Transform3DInverseWithExtrinsics(float* XYZIn, float* XYZOut);
	void Transform3D_Depth2RGB(float* XYZIn, float* XYZOut);
	void ConvertRealWorld3DToProjectiveRGB(const float *X,  float& u, float &v);

	void FromGlobalCSToProjectiveRGB(float *X,  float& u, float &v);
	void FromGlobalCSToLocalAndProjectiveDepthAndRGB(float *XGlobal,  float* XLocal, float& uDepth, float &vDepth, float& uRGB, float &vRGB);

	void FromGlobalCSToLocalNormals(float* nXGlobal, float* nXLocal);
		
	private:
	
};

namespace Nano3D {

/// Buffer-based mesh object (OpenGL compliant). Constructors do not copy data, unless for the copy one of course.
class Mesh {
public:

    class Exception {
    public:
        inline Exception (const char * msg) : _msg (std::string ("[Mesh Exception]") + msg) {}
        inline Exception (std::string & msg) : _msg (std::string ("[Mesh Exception]") + msg) {}
        inline virtual ~Exception () {}
        inline const std::string & msg () const { return _msg; }
    private:
        std::string _msg;
    };

    /// A vertex iterator allowing to navigate the vertex buffer of the mesh.
    class VertexIterator {
    public:
        inline VertexIterator () : _index (0), _data (NULL) {}
        inline VertexIterator (unsigned int index, float * data) : _index (index), _data (data) {}
        inline virtual ~VertexIterator () {}
        inline bool operator!= (const VertexIterator & i) { return (i._index != _index || i._data != _data); }
        inline bool operator== (const VertexIterator & i) { return !(*this != i); }
        inline VertexIterator& operator++ () { _index++; return (*this); }
        inline VertexIterator& operator++ (int) { return (++(*this)); }
        inline operator unsigned int () const { return _index; }
    private:
        unsigned int _index;
        float * _data;
    };

    /// A triangle iterator allowing to navigate the index buffer of the mesh.
    class TriIterator {
    public:
        inline TriIterator () : _index (0), _data (NULL) {}
        inline TriIterator (unsigned int index, unsigned int * data) : _index (index), _data (data) {}
        inline virtual ~TriIterator () {}
        inline bool operator!= (const TriIterator & i) { return (i._index != _index || i._data != _data); }
        inline bool operator== (const TriIterator & i) { return !(*this != i); }
        inline TriIterator& operator++ () { _index++; return (*this); }
        inline TriIterator& operator++ (int) { return (++(*this)); }
        inline unsigned int & operator[] (unsigned int i) { return _data[3*_index+i]; }
    private:
        unsigned int _index;
        unsigned int * _data;
    };

	Mesh (const Mesh & m); 
    Mesh (); 
    Mesh (unsigned int numV, float * V); 
    Mesh (unsigned int numV, float * V, unsigned int numT, unsigned int * T);
     
    inline virtual ~Mesh () { clear (); }
    Mesh& operator= (const Mesh & m);
    inline float * vertices () { return _V; }
    inline const float * vertices () const { return _V; }
    inline unsigned int numV () const { return _numV; }
    inline unsigned int * triangles () { return _T; }
    inline const unsigned int * triangles () const { return _T; }
    inline unsigned int numT () const { return _numT; }
    void clear ();
    void clearVertices ();
    void clearTriangles ();

    // Direct data modification (no copy). Careful, memory to be managed manually	
    void init (unsigned int numV, float * V, unsigned int numT, unsigned int * T, bool copyData, bool clearFirst);
    void init (const Mesh & m, bool copyData, bool clearFirst) {init (m._numV, m._V, m._numT, m._T, copyData, clearFirst); }
	
    // Attribute access
    Vec3Df pos (unsigned int index) const { return Vec3Df (_V[9*index], _V[9*index + 1], _V[9*index + 2]); }
    Vec3Df normal (unsigned int index) const { return Vec3Df (_V[9*index+3], _V[9*index + 4], _V[9*index + 5]); }
    Vec3Df color (unsigned int index) const { return Vec3Df (_V[9*index + 6], _V[9*index + 7], _V[9*index + 8]); }
    void setPos (unsigned int index, const Vec3Df & pos) { for (unsigned int i = 0; i < 3; i++) _V[9*index+i] = pos[i];}
    void setNormal (unsigned int index, const Vec3Df & normal) { for (unsigned int i = 0; i < 3; i++) _V[9*index+3+i] = normal[i];}
    void setColor (unsigned int index, const Vec3Df & color) { for (unsigned int i = 0; i < 3; i++) _V[9*index+6+i] = color[i];}

    // Iterators
    VertexIterator makeVertexIterator (unsigned int index) const { return VertexIterator (index, _V); }
    VertexIterator beginVertexIterator () const { return VertexIterator (0, _V); }
    VertexIterator endVertexIterator () const { return VertexIterator (_numV, _V); }
    TriIterator makeTriIterator (unsigned int index) const { return TriIterator (index, _T); }
    TriIterator beginTriIterator () const { return TriIterator (0, _T); }
    TriIterator endTriIterator () const { return TriIterator (_numT, _T); }

    // Operators
    /// Recompute the normal of each vertex based in its incident faces, uniformly (w = 0), based on areas (w = 1) or angles (w = 2).
    void recomputeSmoothVertexNormals (unsigned int w);
    /// Gather, for each vertex i, the list of its 1-ring neighbors in oneRing[i]
    void collectOneRing (std::vector<std::vector<unsigned int> > & oneRing) const;
    void normalizeVertexNormals ();
    /// Compute the bounding sphere of the mesh.
    void computeAveragePosAndRadius (Vec3Df & center, float & radius) const;
    /// Scale the vertex buffer to the unit [-1,1] cube.
    void scaleToUnitBox (Vec3Df & center, float & scaleToUnit);
    void loadOFF (const std::string & filename, unsigned int normWeight = 2);
    void loadPLY (const std::string & filename);
    void storePLY (const std::string & filename, bool pointSetOnly);
	
	static void normalize_mesh_colors(Nano3D::Mesh *m, float norm_factor){
			float* coords = m->vertices() + 6;
			for(unsigned int i=0; i< m->numV();i++, coords+=6){
				*coords = norm_factor * (*coords); 
				*coords = *coords >= 0 ? *coords  : 0; 
				*coords = *coords > 255 ? 255 :  *coords; 
				coords++;
				*coords = norm_factor * (*coords); 
				*coords = *coords >= 0 ? *coords  : 0; 
				*coords = *coords > 255 ? 255 :  *coords; 
				coords++;
				*coords = norm_factor * (*coords); 
				*coords = *coords >= 0 ? *coords  : 0; 
				*coords = *coords > 255 ? 255 :  *coords; 
				coords++;
			}
	};
	
private:
    unsigned int _numV;
    float * _V;
    unsigned int _numT;
    unsigned int * _T;


	// dalexiad (CERTH) additions 
	// ------------------------------------------------------------------------------------------------------------
public:
	//Texture images related
	inline unsigned int getNumOfTextureImages(){return m_numOfImages;};
	inline bool hasTextureImages(){return m_numOfImages>0;};
	void setTextureImage(unsigned char* inputImage, unsigned int imNx, unsigned int imNy, unsigned int texImageID);

	unsigned char* getTextureImage(unsigned int texImageID, unsigned int &imNx, unsigned int &imNy);
	void GetUVTextureMappingCoords(int texID, unsigned int triangleID, float* u, float* v);
	void GetUVTextureMappingCoordsV(int texID, float* posXYZ, float &u, float &v);

	void SetRGBDepthCameraParams(unsigned int texID, const CRGBDepthCameraParams & params);
	CRGBDepthCameraParams* getRGBDepthCameraParams(unsigned int paramsID);
private:	
	//Texture images		
	unsigned int m_numOfImages;
	unsigned int m_imNx;
	unsigned int m_imNy;
	unsigned char* m_image[MAX_NUM_OF_TEXTURE_IMAGES];
	void clearTextureImages();
public:	
	//CWI compressed texture images (do compression outside) no allocation, should be done outside
	void setCompTextureImage(unsigned char* comp_inputImage, unsigned long buffer_size, unsigned int texImageID);
	unsigned char* getCompTextureImage(unsigned int texImageID, unsigned long &Bufsize);
private:
	long m_comp_image_sizes[MAX_NUM_OF_TEXTURE_IMAGES];
	unsigned char* m_comp_image[MAX_NUM_OF_TEXTURE_IMAGES];
	int m_comp_buffer_count;

	//UV texture mapping related
	CRGBDepthCameraParams m_RGBDepthCameraParams[MAX_NUM_OF_TEXTURE_IMAGES];	

	void copyImages(Mesh& m);
	
	// CWI compressed textures
	public:
		int getNumCompImages(){return m_comp_buffer_count;}
};

}
// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
// --------------------------------------------------------------------------
//
// Copyright(C) 2007-2013
// Tamy Boubekeur
//
// All rights reserved.
//
// --------------------------------------------------------------------------

#ifndef PLYLOADER_H
#define PLYLOADER_H

#include <string>
#include <iostream>
#include <fstream>

namespace Nano3D {

/// PLY loader (v0.5 : read triangular meshes).
class PLY {
public:

    typedef enum {
        BINARY_BIG_ENDIAN_1 = 0,
        BINARY_LITTLE_ENDIAN_1,
        ASCII_1
    } Format;

    class Exception {
    private:
        std::string message;
    public :
        Exception (const std::string & msg) : message (msg) {}
        virtual  ~Exception () {}
        const std::string getMessage () const {
            return "PLY Loader Exception:" + message;
        }
    };

    static void load (const std::string & filename,
                      unsigned int & numOfVertices, unsigned int & numOfFaces,
                      bool & hasNormals, bool & hasColors,
                      char ** vertices, unsigned int ** faces);

    static void save (const std::string & filename, Format format,
                      unsigned int numOfVertices, unsigned int numOfFaces,
                      bool hasNormals, bool hasColors,
                      const float * vertices, const unsigned int * faces);

private:
    static unsigned int readHeader (const std::string & filename,
                                    unsigned int & numOfVertices, unsigned int & numOfFaces,
                                    bool & hasNormals, bool & hasColors, Format & format,
                                    unsigned int & vertexSize);
    static void readBinary1Body (const std::string & filename, unsigned int headerSize, unsigned int numOfVertices,
                                 unsigned int numOfFaces, bool hasNormals, bool hasColors, unsigned int vertexSize,
                                 bool bigEndian, char * vertices, unsigned int * faces);
    static void readASCII1Body (const std::string & filename, unsigned int headerSize, unsigned int numOfVertices,
                                unsigned int numOfFaces, bool hasNormals, bool hasColors, unsigned int vertexSize,
                                char * vertices,  unsigned int * faces);
    static void writeHeader (std::ofstream & out, Format format, unsigned int numOfVertices, unsigned int numOfFaces,
                             bool hasNormals, bool hasColors);
    static void writeBody (std::ofstream & out, Format format,
                           unsigned int numOfVertices, unsigned int numOfFaces,
                           bool hasNormals, bool hasColors,
                           const float * vertices, const unsigned int * faces);
    static void closeStreamAndThrowException (std::ifstream & f, const std::string & msg);
    static void closeStreamAndThrowException (std::ofstream & f, const std::string & msg);
    static unsigned int computeStride (bool hasNormals, bool hasColors);

};

}

#endif // PLYLOADER_H

// Some Emacs-Hints -- please don't remove:
//
// Local Variables:
// mode:C++
// tab-width:4
// End:
#endif