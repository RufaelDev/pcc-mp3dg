/*
* Software License Agreement (BSD License)
*
*  Point Cloud Library (PCL) - www.pointclouds.org
*  Copyright (c) 2014- Centrum Wiskunde en Informatica
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
*   * Neither the name of copyright holder(s)  nor the names of its
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
#include <iostream>
#include <fstream>
#include <vector>

#include <cmath>
#include <pcl/compression_eval/compression_eval.h>

using namespace std;
using namespace Nano3D;

static const unsigned int MAX_COMMENT_SIZE = 256;

#ifdef _MSC_VER
#  define fscanf fscanf_s
#  pragma warning(disable : 4996) //Disable depreciated use of strerror
#endif

#if __cplusplus < 201103L
struct bounding_box
{
    Eigen::Vector4f min_xyz;
    Eigen::Vector4f max_xyz;
};
#endif//__cplusplus


// ***************
// Public methods.
// ***************

void PLY::load (const string & filename,
                unsigned int & numOfVertices,
                unsigned int & numOfFaces,
                bool & hasNormals,
                bool & hasColors,
                char ** vertices,
                unsigned int ** faces) {
    unsigned int vertexSize;
    Format format;
    unsigned int headerSize = readHeader (filename, numOfVertices, numOfFaces, hasNormals, hasColors,
                                          format, vertexSize);
    *vertices = new char[36*numOfVertices];  // keep the size of 9 floats
    if (numOfFaces > 0)
        *faces = new unsigned int[3*numOfFaces];
    else
        *faces = NULL;
    if (format == BINARY_BIG_ENDIAN_1)
        readBinary1Body (filename, headerSize, numOfVertices, numOfFaces, hasNormals, hasColors, vertexSize,
                         true, *vertices, *faces);
    else if (format == BINARY_LITTLE_ENDIAN_1)
        readBinary1Body (filename, headerSize, numOfVertices, numOfFaces, hasNormals, hasColors, vertexSize,
                         false, *vertices, *faces);
    else if (format == ASCII_1)
        readASCII1Body (filename, headerSize, numOfVertices, numOfFaces, hasNormals, hasColors, vertexSize, *vertices, *faces);
    else
        throw Exception ("no support for this PLY format");
	hasNormals = true;
	hasColors = true;
}


void PLY::save (const string & filename, Format format,
                unsigned int numOfVertices, unsigned int numOfFaces,
                bool hasNormals, bool hasColors,
                const float * vertices, const unsigned int * faces) {
    ofstream out (filename.c_str ());
    if (!out)
        throw PLY::Exception ("bad output file");
    writeHeader (out, format, numOfVertices, numOfFaces, hasNormals, hasColors);
    writeBody (out, format, numOfVertices, numOfFaces, hasNormals, hasColors, vertices, faces);
    out.close ();
}



// ****************
// Private methods.
// ****************


unsigned int PLY::readHeader (const string & filename, unsigned int & numOfVertices, unsigned int & numOfFaces,
                              bool & hasNormals, bool & hasColors,
                              Format & format,
                              unsigned int & vertexSize) {
    ifstream in (filename.c_str (), ifstream::in | ifstream::binary);
    if (!in)
        throw Exception ("error opening file");

    hasNormals = false;
    hasColors = false;
    vertexSize = 0;
    bool stillVertex = true;
    string current;
    in >> current;

	if (current != "ply")
        closeStreamAndThrowException (in, "not a PLY file");
    in >> current;
		
	while (current != "end_header") {
        if (current == "format") {
            in >> current;
            if (current == "binary_big_endian") {
                in >> current;
				if (current == "1.0")
                    format = BINARY_BIG_ENDIAN_1;
                else
                    closeStreamAndThrowException (in, "error parsing header - not supported binary big endian version");
            } else if (current == "binary_little_endian") {
                in >> current;
				if (current == "1.0")
                    format = BINARY_LITTLE_ENDIAN_1;
                else
                    closeStreamAndThrowException (in, "error parsing header - not supported binary little endian version");
            } else if (current == "ascii") {
                in >> current;
				if (current == "1.0")
                    format = ASCII_1;
                else
                    closeStreamAndThrowException (in, "error parsing header - not supported ascii version");
            } else
                closeStreamAndThrowException (in, "error parsing header (format)");
        } else if (current == "element") {
            in >> current;
			if (current == "vertex")
                in >> numOfVertices;
            else if (current == "face") {
                in >> numOfFaces;
				stillVertex = false;
            } else if (current == "tristrips") {
                in >> numOfFaces;
				numOfFaces = 0;
                stillVertex = false;
            } else
                closeStreamAndThrowException (in, "error parsing header (element)");
        } else if (current == "property") {
            in >> current;
            if (current == "float" || current == "float32") {
                if (stillVertex == true) 
                    vertexSize += 4;
                in >> current;
                if (current == "nx" || current == "ny" || current == "nz")
                    hasNormals = true;
                else if (current == "red" || current == "green" || current == "blue" ||
                         current == "diffuse_red" || current == "diffuse_green" || current == "diffuse_blue")
                    hasColors = true;
                
            } else if (current == "int" || current == "int32") {
                if (stillVertex == true)
                    vertexSize += 4;
                in >> current;
            } else if (current == "uchar" || current == "uint8") {
                if (stillVertex == true) {
                    hasColors = true;
                    vertexSize++;
                }
                in >> current;
            } else if (current == "list") {
                in >> current;
                in >> current;
                in >> current;
            } else
                closeStreamAndThrowException (in, "error parsing header (property)");
        } else if (current == "comment") {
            char comment[MAX_COMMENT_SIZE];
            in.getline (comment, MAX_COMMENT_SIZE);
        } else
            closeStreamAndThrowException (in, "error parsing header");
        in >> current;
    }
    int headerSize = in.tellg ();
	if (headerSize == -1)
		closeStreamAndThrowException (in, "bad header");
    in.close ();
    return headerSize+1;
}


template <class T>
void bigLittleEndianSwap (T * v, unsigned int numOfElements) {
    char * tmp = (char*)v;
    for (unsigned int j = 0; j < numOfElements; j++) {
        unsigned int offset = 4*j;
        char c = tmp[offset];
        tmp[offset] =  tmp[offset+3];
        tmp[offset+3] = c;
        c = tmp[offset+1];
        tmp[offset+1] = tmp[offset+2];
        tmp[offset+2] = c;
    }
}


void PLY::readBinary1Body (const std::string & filename,
                           unsigned int headerSize,
                           unsigned int numOfVertices,
                           unsigned int numOfFaces,
                           bool hasNormals,
                           bool hasColors,
                           unsigned int vertexSize,
                           bool bigEndian,
                           char * vertices,
                           unsigned int * faces) {
    ifstream in (filename.c_str (), ios::binary);
    if (!in)
        throw Exception ("error opening file");
    for (unsigned int i = 0; i < headerSize; i++) {
            char c;
            in.get (c);
    }
    unsigned int cptVertices = 0;
    char * v = new char[vertexSize];
    for (unsigned int i = 0; i < numOfVertices && !in.eof (); i++) {
        for (unsigned int j = 0; j < vertexSize; j++)
            in.get (v[j]);
        if (bigEndian == true) {
            if (hasNormals && hasColors)
                bigLittleEndianSwap (v, 9);
            else if (hasNormals || hasColors) 
                bigLittleEndianSwap (v, 6);
            else 
                bigLittleEndianSwap (v, 3);
        }
        for (unsigned int j = 0; j < 12; j++)
            vertices[cptVertices++] = v[j];
        if (hasNormals == true)
            for (unsigned int j = 0; j < 12; j++)
                vertices[cptVertices++] = v[12+j];
        if (hasColors == true)
            for (unsigned int j = 0; j < 12; j++)
                vertices[cptVertices++] = v[(hasNormals ? 24 : 12) + j];
    }
    delete [] v;

    if (in.eof())
      closeStreamAndThrowException (in, "incomplete file");

    unsigned int cptFaces = 0;
    for (unsigned int i = 0; i < numOfFaces && !in.eof (); i++) {
        unsigned int f[4];
        char polygonSize;
        in.get(polygonSize);
        for (unsigned int j = 0; j < 12; j++)
            in.get (((char*)f)[j]);
        if (bigEndian == true)
            bigLittleEndianSwap (f, 3);
        for (unsigned int j = 0; j < 3; j++)
            faces[cptFaces++] = static_cast<unsigned int>(f[j]);
    }
    in.close ();
}

void PLY::readASCII1Body (const std::string & filename,
                          unsigned int headerSize,
                          unsigned int numOfVertices,
                          unsigned int numOfFaces,
                          bool hasNormals,
                          bool hasColors,
                          unsigned int vertexSize,
                          char * vertices,
                          unsigned int * faces) {
    ifstream in (filename.c_str ());
	std::string line;
    if (!in)
        throw Exception ("error opening file");
    while (getline(in,line)) {
		if(line.find("end_header")!= string::npos)
			break;
    }
//  unsigned int stride = computeStride (hasNormals, hasColors); // stride is not used and 'computeStride' has no side effects
    unsigned int cptVertices = 0;
    for (unsigned int i = 0; i < numOfVertices && !in.eof (); i++) {
        for (unsigned int j = 0; j < 3; j++) {
            float v;
            in >> v;
            if (j < 3) {
                char * vMask;
                vMask = (char*)(&v);
                for (unsigned int k = 0; k < 4; k++)
                    vertices[cptVertices++] = vMask[k];
            }
        }
		for (unsigned int j = 0; j < 3; j++) {
            float v=0;
			if(hasNormals)
				in >> v;
            
            char * vMask;
            vMask = (char*)(&v);
            for (unsigned int k = 0; k < 4; k++)
               vertices[cptVertices++] = vMask[k];
            
        }
		for (unsigned int j = 0; j < 3; j++) {
			float v=0;
			if(hasColors)
			{
				in >> v; 
				//if ( v > 1.0 ) v=v/(float)255.0; 
			}
            
            char * vMask;
            vMask = (char*)(&v);
            for (unsigned int k = 0; k < 4; k++)
               vertices[cptVertices++] = vMask[k];
            
        }
    }
    if (in.eof ())
        closeStreamAndThrowException (in, "incomplete file");
    unsigned int cptFaces = 0;
    for (unsigned int i = 0; i < numOfFaces && !in.eof (); i++) {
        unsigned int polygonSize;
        in >> polygonSize;
        vector<unsigned int> f (polygonSize);
        for (unsigned int j = 0; j < polygonSize; j++)
            in >> f[j];
        for (unsigned int j = 0; j < 3; j++)
            faces[cptFaces++] = static_cast<unsigned int>(f[j]);
    }
    in.close ();
}


void PLY::writeHeader (ofstream & out, Format format,
                       unsigned int numOfVertices, unsigned int numOfFaces,
                       bool hasNormals, bool hasColors) {
    out << "ply" << endl;
    if (format == ASCII_1)
        out << "format ascii 1.0" << endl;
    else if (format == BINARY_LITTLE_ENDIAN_1)
        out << "format binary_little_endian 1.0" << endl;
    else if (format == BINARY_BIG_ENDIAN_1)
        out << "format binary_big_endian 1.0" << endl;
    else
        closeStreamAndThrowException (out, "no support for this PLY format output");
    out << "element vertex " << numOfVertices<< endl;
    out << "property float x" << endl;
    out << "property float y" << endl;
    out << "property float z" << endl;
    if (hasNormals) {
        out << "property float nx" << endl;
        out << "property float ny" << endl;
        out << "property float nz" << endl;
    }
    if (hasColors) {
        out << "property uchar red" << endl;
        out << "property uchar green" << endl;
        out << "property uchar blue" << endl;
		// out << "property uchar diffuse_red" << endl;
        // out << "property uchar diffuse_green" << endl;
        // out << "property uchar diffuse_blue" << endl;
    }
    out << "element face " << numOfFaces << endl;
    out << "property list uchar int vertex_indices" << endl;
    out << "end_header" << endl;
}


void PLY::writeBody (ofstream & out, Format format,
                     unsigned int numOfVertices, unsigned int numOfFaces,
                     bool hasNormals, bool hasColors,
                     const float * vertices, const unsigned int * faces) {
    unsigned int stride = computeStride (hasNormals, hasColors);
    if (format == ASCII_1) {
        for (unsigned int i = 0; i < numOfVertices; i++) {
            unsigned int offset = stride*i;
            for (unsigned int j = 0; j < stride; j++) {
				if (j >= 6){
					if(std::ceil(vertices[offset+j]) > 0)
						out << static_cast<unsigned int>(std::ceil(vertices[offset+j]));
					else
						out << 0;
				}
				else{
					out << vertices[offset+j];
				}
				if (j == (stride - 1))
                    out << endl;
                else
                    out << " ";
            }
        }
        for (unsigned int i = 0; i < numOfFaces; i++) {
            unsigned int offset = 3*i;
            out << "3 " << faces[offset] << " " << faces[offset+1] << " " << faces[offset+2] << endl;
        }
    } else if (format == BINARY_LITTLE_ENDIAN_1) {
        out.write ((char*)vertices, 4*stride*numOfVertices);
        for (unsigned int i = 0; i < numOfFaces; i++) {
            unsigned int offset = 3*i;
            char polygonSize = 3;
            out.write (&polygonSize, 1);
            out.write ((char*)(faces+offset), 12);
        }
    } else if (format == BINARY_BIG_ENDIAN_1) {
        bigLittleEndianSwap (vertices, stride*numOfVertices);
        out.write ((char*)vertices, 4 * stride*numOfVertices);
        bigLittleEndianSwap (vertices, stride*numOfVertices);
        bigLittleEndianSwap (faces, 3*numOfFaces);
        for (unsigned int i = 0; i < numOfFaces; i++) {
            unsigned int offset = 3*i;
            char polygonSize = 3;
            out.write (&polygonSize, 1);
            out.write ((char*)(faces+offset), 12);
        }
        bigLittleEndianSwap (faces, 3*numOfFaces);
    } else 
        closeStreamAndThrowException (out, "no support for this PLY format output");
}


void PLY::closeStreamAndThrowException (ifstream & f, const string & msg) {
    f.close ();
    throw Exception (msg);
}

void PLY::closeStreamAndThrowException (ofstream & f, const string & msg) {
    f.close ();
    throw Exception (msg);
}

unsigned int PLY::computeStride (bool hasNormals, bool hasColors) {
    unsigned int stride = 3;
    if (hasNormals && hasColors)
        stride = 9;
    else if (hasNormals || hasColors)
        stride = 6;
    return stride;
}

#include <algorithm>
#include <cstring>

using namespace std;
using namespace Nano3D;

static const Vec3Df VERTEX_DEFAULT_COLOR = Vec3Df (0.6f, 0.6f, 0.6f);

void Mesh::init (unsigned int numV,
	float * V,
	unsigned int numT,
	unsigned int * T,
	bool copyData,
	bool clearFirst) 
{
	if (clearFirst)
		clear ();
	_numV = numV;
	_numT = numT;
	if (copyData) {
		if(_numV==0 || _numT ==0)
		{
			_V = new float [9];
			_T = new unsigned int [3];
			//throw std::exception("tpt-mesh_exception");
		}
		else
		{
			_V = new float [9*_numV];
			memcpy (_V, V, 9*_numV*sizeof (float));
			_T = new unsigned int [3*_numT];
			memcpy (_T, T, 3*_numT*sizeof (unsigned int));
		}		
	} else {
		_V = V;
		_T = T;
	}


	// dalexiad (CERTH) additions
	// ------------------------------------
	m_numOfImages=0;
	m_comp_buffer_count=0;
	for (unsigned int n=0;n<MAX_NUM_OF_TEXTURE_IMAGES;n++)
		m_image[n]=0;
	// ------------------------------------
}


void Mesh::clear () {

    clearVertices ();
    clearTriangles ();

	// dalexiad (CERTH) additions
	// ------------------------------------
	clearTextureImages();
	// ------------------------------------
}

void Mesh::clearVertices () {
    if (_V != NULL)
        delete [] _V;
    _numV = 0;
	_V = NULL;
}

void Mesh::clearTriangles () {
    if (_T != NULL)
        delete [] _T;
    _numT = 0;
	_T = NULL;
}

void Mesh::recomputeSmoothVertexNormals (unsigned int normWeight) {
    vector<Vec3Df> triangleNormals;
    for (TriIterator it = beginTriIterator (); it != endTriIterator (); it++) {
        Vec3Df e01 (pos (it[1]) - pos (it[0]));
        Vec3Df e02 (pos (it[2]) - pos (it[0]));
        Vec3Df n (Vec3Df::crossProduct (e01, e02));
        n.normalize ();
        triangleNormals.push_back (n);
    }
    for (VertexIterator it = beginVertexIterator (); it != endVertexIterator (); it++)
        setNormal (it, Vec3Df (0.0, 0.0, 0.0));
    vector<Vec3Df>::iterator itNormal = triangleNormals.begin ();
    for (TriIterator it = beginTriIterator (); it != endTriIterator (); it++, itNormal++)
        for (unsigned int  j = 0; j < 3; j++) {
            VertexIterator vj = makeVertexIterator(it[j]);
            float w = 1.0; // uniform weights
            Vec3Df e0 = pos (it[(j+1)%3]) - pos (it[j]);
            Vec3Df e1 = pos (it[(j+2)%3]) - pos (it[j]);
            if (normWeight == 1) { // area weight
                w = static_cast<float>(Vec3Df::crossProduct (e0, e1).getLength () / 2.0);
            } else if (normWeight == 2) { // angle weight
                e0.normalize ();
                e1.normalize ();
                w = static_cast<float>((2.0 - (Vec3Df::dotProduct (e0, e1) + 1.0)) / 2.0);
            } 
            if (w <= 0.0)
                continue;
            setNormal (vj, normal (it[j]) + (*itNormal) * w);
        }
    normalizeVertexNormals ();
}

void Mesh::normalizeVertexNormals () {
    for (VertexIterator i = beginVertexIterator (); i != endVertexIterator (); i++) {
        Vec3Df n = normal (i);
        if (n != Vec3Df (0.0, 0.0, 0.0)) {
            n.normalize ();
            setNormal (i, n);
        }
    }
}

void Mesh::computeAveragePosAndRadius (Vec3Df & center,
                                       float & radius) const {
    center = Vec3Df (0.0, 0.0, 0.0);
    for (unsigned int i = 0; i < _numV; i++)
        center += pos (i);
    center /= float (_numV);
    radius = 0.0;
    for (unsigned int i = 0; i < _numV; i++) {
        float vDistance = Vec3Df::distance (center, pos (i));
        if (vDistance > radius)
            radius = vDistance;
    }
}

void Mesh::scaleToUnitBox (Vec3Df & center,
                           float & scaleToUnit) {
    computeAveragePosAndRadius (center, scaleToUnit);
    for (unsigned int i = 0; i < _numV; i++)
        setPos (i, Vec3Df::segment (center, pos (i)) / scaleToUnit);
}

void Mesh::collectOneRing (vector<vector<unsigned int> > & oneRing) const {
    oneRing.resize (_numV);
    for (unsigned int i = 0; i < _numT; i++) {
        Mesh::TriIterator ti = makeTriIterator (i);
        for (unsigned int j = 0; j < 3; j++) {
            unsigned int vj = ti[j];
            for (unsigned int k = 1; k < 3; k++) {
                unsigned int vk = ti[(j+k)%3];
                if (find (oneRing[vj].begin (), oneRing[vj].end (), vk) == oneRing[vj].end ())
                    oneRing[vj].push_back (vk);
            }
        }
    }
}

void Mesh::loadOFF (const std::string & filename,
                    unsigned int normWeight) {
     std::ifstream in (filename.c_str ());
     if (!in)
        throw Exception ("OFF Loading Failed");
     std::string offString;
     unsigned int sizeV, sizeT, tmp;
     in >> offString >> sizeV >> sizeT >> tmp;
     float * v = new float[9*sizeV];
     unsigned int * t = new unsigned int[3*sizeT];
     init (sizeV, v, sizeT, t, false, true);
     for (unsigned int i = 0; i < sizeV; i++) {
         in >> _V[9*i] >> _V[9*i+1] >> _V[9*i+2];
         setColor (i, VERTEX_DEFAULT_COLOR);
     }
     int s;
     for (unsigned int i = 0; i < sizeT; i++)
         in >> s >> _T[3*i] >> _T[3*i+1] >> _T[3*i+2];
     in.close ();
     recomputeSmoothVertexNormals (normWeight);
}

void Mesh::loadPLY(const string & filename) {
    clear ();
    char * v = NULL;
    unsigned int * t = NULL;
    bool hasNormals, hasColors;
    PLY::load (filename, _numV, _numT, hasNormals, hasColors, &v, &t);
    _V = (float*)v;
    _T = t;
}

void Mesh::storePLY(const string & filename,
                    bool pointSetOnly) {
    //PLY::save(filename, PLY::BINARY_LITTLE_ENDIAN_1, _numV, _numT, true, true, _V, _T);
	PLY::save(filename, PLY::ASCII_1, _numV, _numT, true, true, _V, _T);
						
}
// ----------------------------------------------------------------------------------------------------------------------------------------------------------




// dalexiad (CERTH) additions
// --------------------------------------------------------------------------------------------------
void Mesh::clearTextureImages()
{
	for (unsigned int n=0;n<m_numOfImages;n++)
	{
		if (m_image[n])
		{
			delete [] m_image[n];
			m_image[n]=0;
		}

	}
	this->m_numOfImages=0;
	m_imNx=0; m_imNy=0;
}


void Mesh::setTextureImage(unsigned char* inputImage, unsigned int imNx, unsigned int imNy, unsigned int texImageID)
{
	if (texImageID>MAX_NUM_OF_TEXTURE_IMAGES-1)
		return; //Do nothing, just return

	if (m_numOfImages>0)//Already have an image
	{
		//Check whether the size is the same with the size of existing images
		if (imNx != m_imNx || imNy != m_imNy)
		{
			printf("Mesh::setTextureImage: Input image size differs from the size of existing texture images\n");
			return; //Do nothing, just return
		}
	}

	//Else, everything OK. Proceed normally

	if (m_image[texImageID])
		delete m_image[texImageID];
	m_image[texImageID] = new unsigned char[3*imNx*imNy];	
	memcpy(m_image[texImageID],inputImage,3*imNx*imNy*sizeof(unsigned char));

	this->m_imNx = imNx;
	this->m_imNy = imNy;

	//Increase the number of images, if needed
	if (m_numOfImages<texImageID+1)
		m_numOfImages=texImageID+1;
}

unsigned char* Mesh::getTextureImage(unsigned int texImageID, unsigned int &imNx, unsigned int &imNy)
{
	if (texImageID<m_numOfImages && m_image[texImageID]!=0)
	{
		imNx = m_imNx;
		imNy = m_imNy;
		return m_image[texImageID];
	}

	//Else, something wrong happened
	imNx = 0;
	imNy = 0;
	return 0;	
}

/* CWI allow storage of compressed image buffers (i.e. jpegs), carefull no input/output assertion and validation, just stores a reference to a compressed buffer (alloc done outside) */
void Mesh::setCompTextureImage(unsigned char* comp_inputImage, unsigned long buffer_size, unsigned int texImageID)
{
	m_comp_image_sizes[texImageID]	= buffer_size;
	m_comp_image[texImageID] = comp_inputImage;
	m_comp_buffer_count++;
}

/* CWI allow storage of compressed image buffers (i.e. jpegs), carefull no input/output assertion and validation, just stores a reference to a compressed buffer (alloc done outside)*/
unsigned char* Mesh::getCompTextureImage(unsigned int texImageID, unsigned long &Bufsize){
	Bufsize = m_comp_image_sizes[texImageID];
	return m_comp_image[texImageID];
}


void Mesh::SetRGBDepthCameraParams(unsigned int texID, const CRGBDepthCameraParams & params)
{
	
	if (texID>=MAX_NUM_OF_TEXTURE_IMAGES)
	{
		printf("SetRGBDepthCameraParams Error: texID>=MAX_NUM_OF_TEXTURE_IMAGES\n");
		return;
	}

	if (texID>=m_numOfImages)
		printf("SetRGBDepthCameraParams warning: texID>=m_numOfImages\n");


	this->m_RGBDepthCameraParams[texID] = params;
}

CRGBDepthCameraParams* Mesh::getRGBDepthCameraParams(unsigned int paramsID)
{
	return &(m_RGBDepthCameraParams[paramsID]);
}



void Mesh::GetUVTextureMappingCoords(int texID, unsigned int triangleID, float* u, float* v)
{
	if (texID<0 || texID>=MAX_NUM_OF_TEXTURE_IMAGES)
	{
		printf("GetUVTextureMappingCoords Error: texID>=MAX_NUM_OF_TEXTURE_IMAGES\n");
		u=0;
		v=0;
		return;
	}


	unsigned int* vertIndexes = _T+3*triangleID;
	float XYZIn[3];
	float U1,V1;
	for (unsigned int vertCounter=0;vertCounter<3;vertCounter++)
	{
		//Get the x,y,z coord in the global coordinate system
		memcpy(XYZIn,_V + 9*vertIndexes[vertCounter],3*sizeof(float));
		
		//Ask for the  x,y pixels on the RGB image
		m_RGBDepthCameraParams[texID].FromGlobalCSToProjectiveRGB(XYZIn,U1,V1);

		u[vertCounter]=U1/this->m_imNx;
		v[vertCounter]=V1/this->m_imNy;
	}
}


void Nano3D::Mesh::GetUVTextureMappingCoordsV( int texID, float* posXYZ, float &u, float &v )
{
	if (texID<0 || texID>=MAX_NUM_OF_TEXTURE_IMAGES)
	{
		printf("GetUVTextureMappingCoords Error: texID>=MAX_NUM_OF_TEXTURE_IMAGES\n");
		u=0;
		v=0;
		return;
	}

	float U1,V1;


	//Ask for the  x,y pixels on the RGB image
	m_RGBDepthCameraParams[texID].FromGlobalCSToProjectiveRGB(posXYZ,U1,V1);

	u=U1/this->m_imNx;
	v=V1/this->m_imNy;

}



void Nano3D::Mesh::copyImages( Mesh& m)
{
	clearTextureImages();


	for(int i=0; i < m.m_numOfImages; i++) {
		unsigned char* pImage;
		unsigned int nX,nY;
		pImage = m.getTextureImage(i,nX,nY);
		setTextureImage(pImage,nX,nY,i);
	}

	for(int i=0;i<m.m_numOfImages;i++) {
		SetRGBDepthCameraParams(i,*m.getRGBDepthCameraParams(i));
	}
}

Mesh& Nano3D::Mesh::operator=( const Mesh & m )
{
	init (m, true, true); 

	copyImages(const_cast<Mesh&>(m)); 
	
	return (*this);
}




Mesh::Mesh()
{
	init (0, NULL, 0, NULL, false, false);
}

Mesh::Mesh( unsigned int numV, float * V )
{
	 init (numV, V, 0, NULL, false, false);
}

Mesh::Mesh (unsigned int numV, float * V, unsigned int numT, unsigned int * T)
{
	init (numV, V, numT, T, false, false);
}



Mesh::Mesh (const Mesh & m)
	:
    _numV(0),
    _V(NULL),
	_numT(0),
    _T(NULL),
	m_numOfImages(0)
{
	init (m, true, false); 
	copyImages(const_cast<Mesh&>(m)); 
}




// --------------------------------------------------------------------------------------------------


#include "memory.h"



//Definition of some commonly used functions
void HConvertProjectiveToNormalized(const float u, const float v, 
	float* A, 
	float &Xn, float &Yn)
{
	float fx = A[0];
	float cx = A[2];
	float fy = A[4];
	float cy = A[5];

	Xn = (u - cx)/fx;
	Yn = -(v - cy)/fy;
}
void HDistortNormalized(const float Xn, const float Yn, float* mD, float &Xd, float &Yd)
{
	float k1=mD[0];
	float k2=mD[1];
	float k3=mD[4];
	float p1=mD[2];
	float p2=mD[3];
	float r2 = Xn*Xn+Yn*Yn;
	float rc = 1 + k1*r2 +k2*r2*r2 + k3*r2*r2*r2;
	float tdx = 2*p1*Xn*Yn + p2*(r2 + 2*Xn*Xn);
	float tdy = 2*p2*Xn*Yn + p1*(r2 + 2*Yn*Yn);
	Xd=rc*Xn+tdx;
	Yd=rc*Yn+tdy;
}

//Without radial distortion
void HConvertProjectiveToRealWorldStandard(float u, float v, float Z, float* A, float* XYZ)
{
	float Xn, Yn;		

	HConvertProjectiveToNormalized(u, v, A, Xn, Yn);	
	XYZ[0] = Xn*Z;
	XYZ[1] = Yn*Z;	
	XYZ[2] = Z;	
}

//With radial distortion
void HConvertProjectiveToRealWorld(float u, float v, float Z, float* A, float* D, float* XYZ)
{
	float Xn, Yn;		

	HConvertProjectiveToNormalized(u, v, A, Xn, Yn);	

	float Xd,Yd;
	HDistortNormalized(Xn, Yn, D, Xd, Yd);

	XYZ[0] = Xd*Z;
	XYZ[1] = Yd*Z;	
	XYZ[2] = Z;	
}

void HTransform3D(float* XYZIn, float* R, float* T, float* XYZOut)
{
	float X=XYZIn[0];
	float Y=XYZIn[1];
	float Z=XYZIn[2];

	for (int j=0;j<3;j++)
		XYZOut[j] = R[3*j+0]*X + R[3*j+1]*Y + R[3*j+2]*Z + T[j];

}

void HTransform3DInverse(float* XYZIn, float* R, float* T, float* XYZOut)
{
	float X=XYZIn[0];
	float Y=XYZIn[1];
	float Z=XYZIn[2];

	X-=T[0];
	Y-=T[1];
	Z-=T[2];

	for (int j=0;j<3;j++)
		XYZOut[j] = R[3*0+j]*X + R[3*1+j]*Y + R[3*2+j]*Z;
}


//No radial distortion
void HConvertRealWorld3DToProjectiveStandard(const float *X,  float* A, float& u, float &v)
{
	float fx = A[0];
	float cx = A[2];
	float fy = A[4];
	float cy = A[5];

	//Normalize
	float Xn=X[0]/X[2];
	float Yn=X[1]/X[2];


	u=fx*Xn+cx;
	v=-fy*Yn+cy;
}


//With radial distortion
void HConvertRealWorld3DToProjective(const float *X,  float* A, float* D, float& u, float &v)
{
	float fx = A[0];
	float cx = A[2];
	float fy = A[4];
	float cy = A[5];

	//Normalize
	float Xn=X[0]/X[2];
	float Yn=X[1]/X[2];

	float Xd, Yd;
	HDistortNormalized(Xn, Yn, D, Xd, Yd);

	u=fx*Xd+cx;
	v=-fy*Yd+cy;
}




CRGBDepthCameraParams::CRGBDepthCameraParams(void)
{
	memset(m_R,0,9*sizeof(float));
	memset(m_T,0,3*sizeof(float));
	memset(m_ARGB,0,9*sizeof(float));
	memset(m_DRGB,0,5*sizeof(float));
	memset(m_depthToRGB_R,0,9*sizeof(float));
	memset(m_depthToRGB_T,0,3*sizeof(float));


	m_hasRGBDistortion = false;
	m_hasRGBDepth_RT = false;
}



void CRGBDepthCameraParams::SetAllParameters(const float* R, const float* T, const float* ADepth, const float* ARGB, const float* DRGB, 
	const float* depthToRGB_R, const float* depthToRGB_T,
	bool hasRGBDistortion, bool hasRGBDepth_RT)
{
	m_hasRGBDistortion=hasRGBDistortion;
	m_hasRGBDepth_RT=hasRGBDepth_RT;



	memcpy(m_R,R,9*sizeof(float));
	memcpy(m_T,T,3*sizeof(float));
	memcpy(m_ARGB,ARGB,9*sizeof(float));
	memcpy(m_ADepth,ADepth,9*sizeof(float));

	if (m_hasRGBDistortion)
		memcpy(m_DRGB,DRGB,5*sizeof(float));

	if (m_hasRGBDepth_RT)
	{
		memcpy(m_depthToRGB_R,depthToRGB_R,9*sizeof(float));
		memcpy(m_depthToRGB_T,depthToRGB_T,3*sizeof(float));
	}	
}


void CRGBDepthCameraParams::Transform3DWithExtrinsics(float* XYZIn, float* XYZOut)
{
	HTransform3D(XYZIn,m_R,m_T,XYZOut);
}

void CRGBDepthCameraParams::Transform3DInverseWithExtrinsics(float* XYZIn, float* XYZOut)
{
	HTransform3DInverse(XYZIn,m_R,m_T,XYZOut);
}

void CRGBDepthCameraParams::Transform3D_Depth2RGB(float* XYZIn, float* XYZOut)
{
	if (!m_hasRGBDepth_RT)
	{
		memcpy(XYZOut,XYZIn,3*sizeof(float));
		return;
	}


	XYZIn[1]*=-1; ////minus
	HTransform3D(XYZIn, m_depthToRGB_R, m_depthToRGB_T, XYZOut);
	XYZIn[1]*=-1; ////minus
	XYZOut[1]*=-1; ////minus
}

void CRGBDepthCameraParams::ConvertRealWorld3DToProjectiveRGB(const float *X,  float& u, float &v)
{
	if (!m_hasRGBDistortion)
		HConvertRealWorld3DToProjectiveStandard(X,  m_ARGB, u, v);
	else
		HConvertRealWorld3DToProjective(X,m_ARGB,m_DRGB,u,v);
}

void CRGBDepthCameraParams::FromGlobalCSToProjectiveRGB(float *X,  float& u, float &v)
{
	float XYZIn1[3];
	float XYZOut[3];

	//Rotate-translate the point to the local coordinate system,...
	Transform3DInverseWithExtrinsics(X, XYZIn1);
	//To the coord system of RGB camera...
	Transform3D_Depth2RGB(XYZIn1, XYZOut);
	//And project to 2D to get the UV coord
	ConvertRealWorld3DToProjectiveRGB(XYZOut,  u, v);
}


void CRGBDepthCameraParams::FromGlobalCSToLocalAndProjectiveDepthAndRGB(float *XGlobal,  float* XLocal, float& uDepth, float &vDepth, float& uRGB, float &vRGB)
{

	float XYZOut[3];

	//Rotate-translate the point to the local coordinate system,...
	Transform3DInverseWithExtrinsics(XGlobal, XLocal);

	HConvertRealWorld3DToProjectiveStandard(XLocal,  m_ADepth, uDepth, vDepth);

	//To the coord system of RGB camera...
	Transform3D_Depth2RGB(XLocal, XYZOut);
	//And project to 2D to get the UV coord
	ConvertRealWorld3DToProjectiveRGB(XYZOut,  uRGB, vRGB);
}

void CRGBDepthCameraParams::FromGlobalCSToLocalNormals( float* nXGlobal, float* nXLocal )
{
	float zeroT[3]={0};
	HTransform3DInverse(nXGlobal,m_R,zeroT,nXLocal);
}

void CRGBDepthCameraParams::ConvertProjectiveDepthToRealWorldRGB( float u, float v, float Z, float *X )
{
	//No radial distortion is considered for depth camera

	float XYZDepth[3];
	HConvertProjectiveToRealWorldStandard(u, v, Z, m_ADepth, XYZDepth);
	
	//Transform with Depth2RGB RT, to go to the coordinate system of the RGB camera
	this->Transform3D_Depth2RGB(XYZDepth,X);
}

void CRGBDepthCameraParams::SetEYEExtrinsics()
{
	memset(m_R,0,9*sizeof(float));
	m_R[0]=m_R[4]=m_R[8]=1;	
	memset(m_T,0,3*sizeof(float));
}
#include <pcl/compression_eval/compression_eval.h>
#include <pcl/compression_eval/impl/compression_eval_impl.hpp>

#include <pcl/quality/quality_metrics.h>
#include <pcl/quality/impl/quality_metrics_impl.hpp>

#include <boost/program_options.hpp>
#include<boost/program_options/parsers.hpp>

#include <assert.h>
#include <sstream>
#include <utility>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>

using namespace std;
using namespace pcl;
using namespace pcl::quality;
using namespace pcl::io;
using namespace pcl::octree;
using namespace pcl::console;

namespace po = boost::program_options;
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**! 
* \struct to store meshes metadata for official evaluation by MPEG committee
* \author Rufael Mekuria (rufael.mekuria@cwi.nl)
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct compression_eval_mesh_meta_data
{
  string original_file_name;
  size_t original_file_size;
  bool has_coords;
  bool has_normals;
  bool has_colors;
  bool has_texts;
  bool has_conn;
  compression_eval_mesh_meta_data()
    : has_coords(true),  has_normals(false), has_colors(false), has_texts(false), has_conn(false)
  {}
};

//! explicit instantiation of the octree compression modules from pcl
//template class OctreePointCloudCodecV2<PointXYZRGB>;

template class PCL_EXPORTS pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>;
template class PCL_EXPORTS pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>;

typedef OctreePointCloudCompression<PointXYZ> pointsOnlyOctreeCodec;
typedef OctreePointCloudCompression<PointXYZRGB> colorOctreeCodec;
typedef OctreePointCloudCodecV2<PointXYZRGB> colorOctreeCodecV2;
///////////////  Bounding Box Logging /////////////////////////	
// log information on the bounding boxes, which is critical for alligning clouds in time
ofstream bb_out("bounding_box_pre_mesh.txt");
Eigen::Vector4f min_pt_bb;
Eigen::Vector4f max_pt_bb;
bool is_bb_init=false;
double bb_expand_factor = 0.10;
/////////////// END CODEC PARAMETER SETTINGS /////////////////////////

void
  printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input_dir1 input_dir2 ............ input_dirN\n put the parameter_config.txt", argv[0]);
}

//! function for loading a mesh file
bool
  loadPLYMesh (const string &filename, pcl::PolygonMesh &mesh)
{
  TicToc tt;
  print_highlight ("Loading "); 
  print_value ("%s ", filename.c_str ());
  pcl::PLYReader reader;
  tt.tic ();

  if (reader.read (filename, mesh) < 0)
    return (false);

  print_info ("[done, "); 
  print_value ("%g", tt.toc ()); 
  print_info (" ms : "); 
  print_value ("%d", mesh.cloud.width * mesh.cloud.height); 
  print_info (" points]\n");
  print_info ("Available dimensions: "); 
  print_value ("%s\n", pcl::getFieldsList (mesh.cloud).c_str ());

  return (true);
}

//! function for loading a mesh files in a folder
bool
  loadPLYFolder(const string &folder_name, vector<pcl::PolygonMesh> &meshes, vector<compression_eval_mesh_meta_data> &meshes_meta_data){

    // check if folder is directory
    if(!boost::filesystem::is_directory(folder_name)){
      print_info("::LoadPLYFolder: not a directory!"); 
      print_value("%s\n", folder_name.c_str());
      return false;
    }

    // use boost file_system to load all the mesh files in the folder to the meshes array
    boost::filesystem::directory_iterator dir_iter(folder_name);
    boost::filesystem::directory_iterator end_iter;

    for(  ; dir_iter != end_iter ; ++dir_iter)
    {
      if(boost::filesystem::is_regular_file(dir_iter->path()))
      {
        string file_name = dir_iter->path().generic_string();
        string file_ext = dir_iter->path().extension().generic_string();

        //! we only support .ply meshes
        if(file_ext ==".ply"){

          //! load the mesh data
          print_info("::LoadFolder: found ply file, "); 
          print_value(" %s file_extension %s \n", file_name.c_str(),file_ext.c_str());
          meshes.push_back(pcl::PolygonMesh());
          loadPLYMesh(file_name,meshes.back());
          //~ done loading mesh

          // load the metadata 
          compression_eval_mesh_meta_data mdata;
          mdata.original_file_name = dir_iter->path().filename().generic_string();
          mdata.original_file_size =  file_size(dir_iter->path());

          //! check if the mesh is a point cloud or not
          if(meshes.back().polygons.size() > 0)
            mdata.has_conn = true;
          else
            mdata.has_conn = false;

          //! check the fields in the point cloud to detect properties of the mesh
#if __cplusplus >= 201103L
          for( auto it = meshes.back().cloud.fields.begin(); it != meshes.back().cloud.fields.end(); ++it)
#else
          for( std::vector<pcl::PCLPointField>::iterator it = meshes.back().cloud.fields.begin(); it != meshes.back().cloud.fields.end(); ++it)
#endif//__cplusplus >= 201103L
          {
            if( it->name == "rgb")
              mdata.has_colors = true;
            if( it->name == "normal_x")
              mdata.has_colors = true;
            if( it->name == "x")
              mdata.has_coords = true;
          }
          meshes_meta_data.push_back(mdata);
          //! done loading metadata
        }
      }
    }
    if(meshes.size())
      return true;
    else 
      return false;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**! 
* \brief 
script for point cloud codec evaluation by MPEG committee
\param  the input command line arguments dir1 , dir2, dir3 ..... 
\note clouds from dir1 , dir2 , dir3 .... will be fused in a single cloud if dir1,..dir2, contain the per-view clouds
* \author Rufael Mekuria (rufael.mekuria@cwi.nl)
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
  main (int argc, char** argv)
{
  print_info ("Load a Folder of Point Clouds\n ", argv[0]);

  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }

  ////////////////// parse configuration settings from ..//parameter_config.txt ///////////////////
  boost::program_options::options_description desc;
  desc.add_options()
    ("help", " produce help message ")
    ("mesh_file_folders", po::value<vector<string> >(), " folder mesh files ")
    ("octree_bit_settings", po::value<vector<int> >(), " quantization bit assignment octree ")
    ("color_bit_settings", po::value<vector<int> >(), "color bit assignment octree or jpeg quality values ")
    ("enh_bit_settings", po::value<int>(), " bits to code the points towards the center ")
    ("color_coding_types", po::value<vector<int> >(), "  pcl=0,jpeg=1 or graph transform ")
    ("keep_centroid", po::value<int>()->default_value(1), " for keeping centroid ")
    ("bb_expand_factor", po::value<double>()->default_value(0.15), " bounding box expansion to keep bounding box accross frames ")
    ("output_csv_file", po::value<string>()->default_value("bench_out.csv")," output .csv file ")
    ("write_output_ply", po::value<int>()->default_value(0)," write output as .ply files")
    ("do_delta_frame_coding", po::value<int>()->default_value(0)," do_delta_frame_coding ")
    ("icp_on_original", po::value<int>()->default_value(0)," icp_on_original ")
    ("pframe_quality_log",po::value<string>()->default_value("pframe_log.csv"), " write the quality results of predictive coding of p frames")
    ("macroblocksize",po::value<int>()->default_value(16), " size of macroblocks used for predictive frame (has to be a power of 2)")
    ("testbbalign",po::value<int>()->default_value(0), " set this option to test allignements only ")
    ;

  po::variables_map vm;
  ifstream in_conf("..//parameter_config.txt");
  po::store(po::parse_config_file(in_conf, desc), vm);
  po::notify(vm);  
  bb_expand_factor = vm["bb_expand_factor"].as<double>();
  ////////////////// ~end parse configuration file  /////////////////////////////////////


  ////////////////// LOADING CLOUDS INTO MEMORY /////////////////////////////////////
  // store folders to load in a vector
  vector<int> ply_folder_indices = parse_file_extension_argument (argc, argv, "");

  // store all loaded meshes in a vector and store all metadata separately (optional)
  vector<vector<pcl::PolygonMesh> > meshes;
  vector<vector<compression_eval_mesh_meta_data> > meshes_meta_data;

  // data structures for storing the fused meshes
  vector<boost::shared_ptr<pcl::PointCloud<PointXYZRGB> > > fused_clouds;
  vector<compression_eval_mesh_meta_data> fused_clouds_meta_data;

  meshes.resize(ply_folder_indices.size());
  meshes_meta_data.resize(ply_folder_indices.size());

  // load all the meshes from the folder (point clouds)
  if(ply_folder_indices.size() > 0)
  {
    for(int i=0; i<ply_folder_indices.size();i++){
      if (!loadPLYFolder (argv[ply_folder_indices[i]], meshes[i], meshes_meta_data[i])) 
      {
        print_info (" Failed to Load Mesh File Folder"); print_value ("%s\n", (argv[ply_folder_indices[0]]));
        return (-1);
      }
    }
  }
  ////////////////// END LOADING CLOUDS //////////////////////////////////////////////////




  //////////////// FUSE CLOUDS ///////////////////////////////////////////////////////

  // in this cycle we create the fused point clouds when more than one folder is added
  if(ply_folder_indices.size() > 0)
  {
    // for each folder (we assume folder have the same number of files and the same ordering)
    for(int j =0; j<ply_folder_indices.size(); j++){
      // for each mesh in first folder create the fused cloud by loading from first folder and appending the rest
      for(int i=0; i <meshes[j].size(); i++){
        colorOctreeCodec::PointCloudPtr l_ptr= colorOctreeCodec::PointCloudPtr(new colorOctreeCodec::PointCloud()); 

        // convert to the point cloud 1 from the blob, with and without colors
        pcl::fromPCLPointCloud2(meshes.at(j).at(i).cloud,*l_ptr);

        // for the first folder, create the fused clouds, append the clouds from the next folders
        if(j == 0)
        {
          fused_clouds.push_back(l_ptr);
        }
        else
        {
          for(int k=0;k<l_ptr->size();k++)
            fused_clouds[i]->push_back((*l_ptr)[k]); // appends the points
        }
      }
    }
  }

  //////////////// END FUSE CLOUDS ///////////////////////////////////////////////////////

  //////////////// Logging of Prediction Performance /////////////////////////////////
  int bb_align_count=0;
  std::vector<bool> aligned_flags(fused_clouds.size()); // flags to check if a cloud is aligned 
  /////////////////////////////////////////////////////////////////////////////////////

  /////////////// NORMALIZE CLOUDS ///////////////////////////////////////////////////////

#if __cplusplus >= 201103L
  struct bounding_box
  {
    Eigen::Vector4f min_xyz;
    Eigen::Vector4f max_xyz;
  };
#endif//__cplusplus

  // initial bounding box
  min_pt_bb[0]= 1000;
  min_pt_bb[1]= 1000;
  min_pt_bb[2]= 1000;

  max_pt_bb[0]= -1000;
  max_pt_bb[1]= -1000;
  max_pt_bb[2]= -1000;
    
  vector<bounding_box> assigned_bbs(fused_clouds.size());
    
  for(int k=0;k<fused_clouds.size();k++){
    /*
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;

    pcl::getMinMax3D<pcl::PointXYZRGB>(*(fused_clouds[k]),min_pt,max_pt);

    bb_out << "[ " << min_pt.x() << "," << min_pt.y() << "," << min_pt.z() << "]    [" << max_pt.x() << "," << max_pt.y() << "," << max_pt.z() <<"]" << endl;

    // check if min fits bounding box, otherwise adapt the bounding box
    if( !((min_pt.x() > min_pt_bb.x()) && (min_pt.y() > min_pt_bb.y()) && (min_pt.z() > min_pt_bb.z())))
    {
      is_bb_init = false;
    }

    // check if max fits bounding box, otherwise adapt the bounding box
    if(!((max_pt.x() < max_pt_bb.x()) && (max_pt.y() < max_pt_bb.y()) && (max_pt.z() < max_pt_bb.z())))
    {
      is_bb_init = false;
    }


    if(!is_bb_init)
    {
      // initialize the bounding box, with bb_expand_factor extra
      assigned_bbs[k].min_xyz[0] = min_pt[0] - bb_expand_factor*abs(max_pt[0] - min_pt[0]);
      assigned_bbs[k].min_xyz[1] = min_pt[1] - bb_expand_factor*abs(max_pt[1] - min_pt[1]);
      assigned_bbs[k].min_xyz[2] = min_pt[2] - bb_expand_factor*abs(max_pt[2] - min_pt[2]);

      min_pt_bb=assigned_bbs[k].min_xyz ;

      assigned_bbs[k].max_xyz[0] = max_pt[0] + bb_expand_factor*abs(max_pt[0] - min_pt[0]);
      assigned_bbs[k].max_xyz[1] = max_pt[1] + bb_expand_factor*abs(max_pt[1] - min_pt[1]);
      assigned_bbs[k].max_xyz[2] = max_pt[2] + bb_expand_factor*abs(max_pt[2] - min_pt[2]);

      max_pt_bb=assigned_bbs[k].max_xyz;

      is_bb_init = true;
      bb_align_count++;
      cout << "re-intialized bounding box !!! " << endl;
      aligned_flags[k] = false;
    }
    else
    {
      assigned_bbs[k].min_xyz= min_pt_bb;
      assigned_bbs[k].max_xyz= max_pt_bb;
      aligned_flags[k] = true;
    }

    Eigen::Vector4f dyn_range = assigned_bbs[k].max_xyz - assigned_bbs[k].min_xyz;

    for(int j=0; j < fused_clouds[k]->size();j++)
    {
      // offset the minimum value
      fused_clouds[k]->at(j).x-=assigned_bbs[k].min_xyz[0];
      fused_clouds[k]->at(j).y-=assigned_bbs[k].min_xyz[1];
      fused_clouds[k]->at(j).z-=assigned_bbs[k].min_xyz[2];

      // dynamic range
      fused_clouds[k]->at(j).x/=dyn_range[0];
      fused_clouds[k]->at(j).y/=dyn_range[1];
      fused_clouds[k]->at(j).z/=dyn_range[2];
    }

    // bounding box is expanded
    Eigen::Vector4f min_pt_res;
    Eigen::Vector4f max_pt_res;

    pcl::getMinMax3D<pcl::PointXYZRGB>(*(fused_clouds[k]),min_pt_res,max_pt_res);

    assert(min_pt_res[0] >= 0);
    assert(min_pt_res[1] >= 0);
    assert(min_pt_res[2] >= 0);

    assert(max_pt_res[0] <= 1);
    assert(max_pt_res[1] <= 1);
    assert(max_pt_res[2] <= 1);
    */
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;

    pcl::getMinMax3D<pcl::PointXYZRGB>(*(fused_clouds[k]),min_pt,max_pt);

    bb_out << "[ " << min_pt.x() << "," << min_pt.y() << "," << min_pt.z() << "]    [" << max_pt.x() << "," << max_pt.y() << "," << max_pt.z() <<"]" << endl;

    // check if min fits bounding box, otherwise adapt the bounding box
    if( !( (min_pt.x() > min_pt_bb.x()) && (min_pt.y() > min_pt_bb.y()) && (min_pt.z() > min_pt_bb.z())))
    {
      is_bb_init = false;
    }

    // check if max fits bounding box, otherwise adapt the bounding box
    if(!((max_pt.x() < max_pt_bb.x()) && (max_pt.y() < max_pt_bb.y()) && (max_pt.z() < max_pt_bb.z())))
    {
      is_bb_init = false;
    }


    if(!is_bb_init)
    {
      aligned_flags[k] = false;
      bb_align_count++;
      // initialize the bounding box, with bb_expand_factor extra
      min_pt_bb[0] = min_pt[0] - bb_expand_factor*abs(max_pt[0] - min_pt[0]);
      min_pt_bb[1] = min_pt[1] - bb_expand_factor*abs(max_pt[1] - min_pt[1]);
      min_pt_bb[2] = min_pt[2] - bb_expand_factor*abs(max_pt[2] - min_pt[2]);

      max_pt_bb[0] = max_pt[0] + bb_expand_factor*abs(max_pt[0] - min_pt[0]);
      max_pt_bb[1] = max_pt[1] + bb_expand_factor*abs(max_pt[1] - min_pt[1]);
      max_pt_bb[2] = max_pt[2] + bb_expand_factor*abs(max_pt[2] - min_pt[2]);

      is_bb_init = true;

      cout << "re-intialized bounding box !!! " << endl;
    }
    else 
      aligned_flags[k] = true;

#if __cplusplus >= 201103L
    auto dyn_range = max_pt_bb - min_pt_bb;
#else
    Eigen::Vector4f  dyn_range = max_pt_bb - min_pt_bb;
#endif//__cplusplus >= 201103L

    assigned_bbs[k].max_xyz = max_pt_bb;
    assigned_bbs[k].min_xyz = min_pt_bb;

    for(int j=0; j < fused_clouds[k]->size();j++)
    {
      // offset the minimum value
      fused_clouds[k]->at(j).x-=min_pt_bb[0];
      fused_clouds[k]->at(j).y-=min_pt_bb[1];
      fused_clouds[k]->at(j).z-=min_pt_bb[2];

      // dynamic range
      fused_clouds[k]->at(j).x/=dyn_range[0];
      fused_clouds[k]->at(j).y/=dyn_range[1];
      fused_clouds[k]->at(j).z/=dyn_range[2];
    }


    // bounding box is expanded

    Eigen::Vector4f min_pt_res;
    Eigen::Vector4f max_pt_res;

    pcl::getMinMax3D<pcl::PointXYZRGB>(*(fused_clouds[k]),min_pt_res,max_pt_res);

    assert(min_pt_res[0] >= 0);
    assert(min_pt_res[1] >= 0);
    assert(min_pt_res[2] >= 0);

    assert(max_pt_res[0] <= 1);
    assert(max_pt_res[1] <= 1);
    assert(max_pt_res[2] <= 1);
  }

  /////////////// END NORMALIZE CLOUDS ///////////////////////////////////////////////////////

  /////////////// PREPARE OUTPUT CSV FILE AND CODEC PARAMTER SETTINGS /////////////////////////
  string o_log_csv = vm["output_csv_file"].as<string>();
#if __cplusplus >= 201103L
  ofstream res_base_ofstream(o_log_csv);
#else
    ofstream res_base_ofstream(o_log_csv.c_str());
#endif//__cplusplus >= 201103L
  string p_log_csv = vm["pframe_quality_log"].as<string>();
#if __cplusplus >= 201103L
  ofstream res_p_ofstream(p_log_csv);
#else
    ofstream res_p_ofstream(p_log_csv.c_str());
#endif//__cplusplus >= 201103L
  ofstream res_enh_ofstream("results_enh.csv");

  // print the headers
  QualityMetric::print_csv_header(res_base_ofstream);
  QualityMetric::print_csv_header(res_p_ofstream);

  /////////////// END PREPARE OUTPUT CSV FILE AND CODEC PARAMTER SETTINGS /////////////////////////

  ////////////// FOR EACH PARAMETER SETTING DO ASSESMENT //////////////////
  int enh_bit_settings = vm["enh_bit_settings"].as<int>();
  vector<int> octree_bit_settings = vm["octree_bit_settings"].as<vector<int> >();
  vector<int> color_bit_settings =  vm["color_bit_settings"].as<vector<int> >();
  vector<int> color_coding_types =  vm["color_coding_types"].as<vector<int> >();
  bool keep_centroid = vm["keep_centroid"].as<int>();
  int write_out_ply =  vm["write_output_ply"].as<int>();
  int do_delta_coding = vm["do_delta_frame_coding"].as<int>();
  int icp_on_original = vm["icp_on_original"].as<int>();
  int macroblocksize= vm["macroblocksize"].as<int>();
  int testbbalign = vm["testbbalign"].as<int>();  // testing the bounding box alignment algorithm

  if(testbbalign){
    std::cout << " re-alligned " << bb_align_count << " frames out of " << fused_clouds.size() <<" frames" << std::endl;
    return true;
  }
  //////////////////////////////////////////////////////////////////////////

  // base layer resolution
  for(int ct=0; ct < color_coding_types.size();ct++ ){

    vector<float> icp_convergence_percentage(fused_clouds.size()); // field store the percentage of converged macroblocks
    vector<float> shared_macroblock_percentages(fused_clouds.size()); // field to store the percentage of macroblocks shared with the previous frame

    // enh layer resolution
    for(int ob=0; ob < octree_bit_settings.size(); ob++){

      // color resolution
      for(int cb=0; cb < color_bit_settings.size(); cb++){

        // store the parameters in a string to store them in the .csv file
        stringstream compression_arg_ss; 
        compression_arg_ss << octree_bit_settings[ob] << "_"  
          <<  color_bit_settings[cb] 
        << "_colort-" << color_coding_types[ct] << "_centroid-" << (keep_centroid ? "yes" : "no");

        ////////////// ASSESMENT: ENCODE, DECODE AND RECORD THE ACHIEVED QUALITY //////////////////

        // declare codecs outside the mesh iterator loop to test double buffering

        //! encode the fused cloud with and without colors
#if __cplusplus >= 201103L
        auto l_codec_encoder = generatePCLOctreeCodecV2<PointXYZRGB>(
#else
        boost::shared_ptr<OctreePointCloudCodecV2<PointXYZRGB> > l_codec_encoder = generatePCLOctreeCodecV2<PointXYZRGB>(
#endif//__cplusplus < 201103L
          octree_bit_settings[ob],
          enh_bit_settings,
          color_bit_settings[cb],
          0,
          color_coding_types[ct],
          keep_centroid
          );

        // set the macroblocksize for inter prediction
        l_codec_encoder->setMacroblockSize(macroblocksize);

        // initialize structures for decoding base and enhancement layers
#if __cplusplus >= 201103L
          auto l_codec_decoder_base = generatePCLOctreeCodecV2<PointXYZRGB>(
#else
          boost::shared_ptr<OctreePointCloudCodecV2<PointXYZRGB> > l_codec_decoder_base = generatePCLOctreeCodecV2<PointXYZRGB>(
#endif//__cplusplus >= 201103L
          octree_bit_settings[ob],
          enh_bit_settings,
          color_bit_settings[cb],
          0,
          color_coding_types[ct],
          keep_centroid
          );

        for(int i=0; i < fused_clouds.size(); i++)
        {
          // structs for storing the achieved quality
          TicToc tt;
          pcl::quality::QualityMetric achieved_quality;
          pcl::quality::QualityMetric pframe_quality;

          //! full compression into stringstreams, base and enhancement layers, with and without colors
          stringstream l_output_base;

          /////////////////////////////////////////////////////////////
          //! do the encoding
          tt.tic ();
          l_codec_encoder->encodePointCloud(fused_clouds[i] ,l_output_base);
          achieved_quality.encoding_time_ms = tt.toc();
          ////////////////////////////////////////////////////////////

          ////////////////////////////////////////////////////////////////
          // store and display the partial bytes sizes
          uint64_t *c_sizes = l_codec_encoder->getPerformanceMetrics();
          achieved_quality.byte_count_octree_layer = c_sizes[0];
          achieved_quality.byte_count_centroid_layer = c_sizes[1];
          achieved_quality.byte_count_color_layer= c_sizes[2];
          ////////////////////////////////////////////////////////////////

          cout << " octreeCoding " << (achieved_quality.compressed_size=l_output_base.tellp()) << " bytes  base layer  " << endl;
          //////////////////////////////////////////////////////////////

          //////////////////// octree delta frame encoding /////////////////////
          // predicted frame, lossy prediction with artefacts that need to be assessed
          boost::shared_ptr<pcl::PointCloud<PointXYZRGB> > out_d(new pcl::PointCloud<PointXYZRGB>());
          if(do_delta_coding){
            if(aligned_flags[i+1]){ // only do delta coding when frames are aligned
              cout << " delta coding frame nr " << i << endl;
              if(i < (fused_clouds.size() -1)) 
              {
                stringstream p_frame_pdat;
                stringstream p_frame_idat;
                // code a delta frame, either use original or simplified cloud for ICP
                tt.tic();
                l_codec_encoder->generatePointCloudDeltaFrame(icp_on_original ? fused_clouds[i] : l_codec_encoder->getOutputCloud(),
                  fused_clouds[i+1],out_d, p_frame_idat, p_frame_pdat, (int) icp_on_original);
                pframe_quality.encoding_time_ms = tt.toc();
                pframe_quality.byte_count_octree_layer = p_frame_idat.tellp();
                pframe_quality.byte_count_centroid_layer = p_frame_pdat.tellp();
                pframe_quality.compressed_size = p_frame_idat.tellp() + p_frame_pdat.tellp();
                pframe_quality.byte_count_color_layer= 0;
                cout << " encoded a predictive frame: coded " << p_frame_idat.tellp() << " bytes intra and " << p_frame_pdat.tellp()  << " inter frame encoded " <<endl;
                
                shared_macroblock_percentages[i+1] = l_codec_encoder->getMacroBlockPercentage();
                icp_convergence_percentage[i+1] = l_codec_encoder->getMacroBlockConvergencePercentage();

                // compute the quality of the resulting predictive frame
                computeQualityMetric<pcl::PointXYZRGB>(*fused_clouds[i+1],*out_d, pframe_quality);
                pframe_quality.print_csv_line(compression_arg_ss.str(), res_p_ofstream);
              }
              // store the quality metrics for the p cloud
            }
          }
          ///////////////////////////////////////////////////////////////////////

          // start decoding and computing quality metrics
          stringstream oc(l_output_base.str());
          colorOctreeCodec::PointCloudPtr decoded_cloud_base = colorOctreeCodec::PointCloudPtr(new colorOctreeCodec::PointCloud);

          // do the decoding base layer
          cout << "starting decoding the point cloud \n" << endl;
          tt.tic ();
          l_codec_decoder_base->decodePointCloud(oc,decoded_cloud_base);
          achieved_quality.decoding_time_ms = tt.toc ();
          cout << "finished decoding the point cloud \n" << endl;
          // end do the decoding base layer

          // compute quality metric of the base layer
          computeQualityMetric<pcl::PointXYZRGB>(*fused_clouds[i],*decoded_cloud_base, achieved_quality);

          if(write_out_ply )
          {
            bool write_rev_format = true;
            if(!write_rev_format ){
              // write the .ply file by converting to point cloud2 and then to polygon mesh
              pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
              pcl::toPCLPointCloud2( *decoded_cloud_base, *cloud2);
              pcl::PLYWriter writer;
              writer.write("ct_" +
                boost::lexical_cast<string>(ct) + 
                "ob_" +
                boost::lexical_cast<string>(ob) + 
                "_cb_" +
                boost::lexical_cast<string>(cb) + 
                "_mesh_nr_" +
                boost::lexical_cast<string>(i) +
                "_out.ply", cloud2
                );
              // end writing .ply
            }
            else{
              ///////////////////////////////////////////////////////
              float *mdat=new float[9*decoded_cloud_base->size()];
              unsigned int *l_triang = new unsigned int[3];
              
              l_triang[0] = 0;
              l_triang[1] = 1;
              l_triang[2] = 2;
              
              Eigen::Vector4f l_range_scale =assigned_bbs[i].max_xyz - assigned_bbs[i].min_xyz;

              Nano3D::Mesh m(decoded_cloud_base->size(),mdat,1,l_triang);
              for(int l=0; l<decoded_cloud_base->size();l++)
              {
                mdat[9*l] = (*decoded_cloud_base)[l].x * l_range_scale[0]  + assigned_bbs[i].min_xyz[0];
                mdat[9*l+1] = (*decoded_cloud_base)[l].y * l_range_scale[1] + assigned_bbs[i].min_xyz[1];
                mdat[9*l+2] = (*decoded_cloud_base)[l].z * l_range_scale[2] + assigned_bbs[i].min_xyz[2];
                mdat[9*l+3] = 0;
                mdat[9*l+4] = 0;
                mdat[9*l+5] = 0;
                mdat[9*l+6] = (*decoded_cloud_base)[l].r;
                mdat[9*l+7] = (*decoded_cloud_base)[l].g;
                mdat[9*l+8] = (*decoded_cloud_base)[l].b; 
              }
              //////////////////////////////////////////////////////
              m.recomputeSmoothVertexNormals(.3);
              m.storePLY("rev_ct_" +
                boost::lexical_cast<string>(ct) + 
                "ob_" +
                boost::lexical_cast<string>(ob) + 
                "_cb_" +
                boost::lexical_cast<string>(cb) + 
                "_mesh_nr_" +
                boost::lexical_cast<string>(i) +
                "_out.ply",false);
            }
            // write predictevely encoded frames
            if(do_delta_coding && aligned_flags[i+1])
            {
              if(!write_rev_format ){
                pcl::PCLPointCloud2::Ptr cloud2d(new pcl::PCLPointCloud2());
                pcl::toPCLPointCloud2( *out_d, *cloud2d);
                pcl::PLYWriter writer;
                writer.write("ct_" +
                  boost::lexical_cast<string>(ct) + 
                  "ob_" +
                  boost::lexical_cast<string>(ob) + 
                  "_cb_" +
                  boost::lexical_cast<string>(cb) + 
                  "_mesh_nr_" +
                  boost::lexical_cast<string>(i+1) +
                  "_out_predicted.ply", cloud2d
                  );
              }
              else
              {
                ///////////////////////////////////////////////////////
                float *mdat=new float[9*out_d->size()];
                unsigned int *l_triang = new unsigned int[3];
                l_triang[0] = 0;
                l_triang[1] = 1;
                l_triang[2] = 2;
                Nano3D::Mesh m(out_d->size(),mdat,1,l_triang);

                Eigen::Vector4f l_range_scale =assigned_bbs[i].max_xyz - assigned_bbs[i].min_xyz;

                for(int l=0; l<out_d->size();l++)
                {
                  mdat[9*l] = (*out_d)[l].x * l_range_scale[0]  + assigned_bbs[i].min_xyz[0];
                  mdat[9*l+1] = (*out_d)[l].y * l_range_scale[1]  + assigned_bbs[i].min_xyz[1];
                  mdat[9*l+2] = (*out_d)[l].z* l_range_scale[2]  + assigned_bbs[i].min_xyz[2];
                  mdat[9*l+3] = 0;
                  mdat[9*l+4] = 0;
                  mdat[9*l+5] = 0;
                  mdat[9*l+6] = (*out_d)[l].r;
                  mdat[9*l+7] = (*out_d)[l].g;
                  mdat[9*l+8] = (*out_d)[l].b; 
                }
                //////////////////////////////////////////////////////
                m.recomputeSmoothVertexNormals(.3);
                m.storePLY("prev_ct_" +
                  boost::lexical_cast<string>(ct) + 
                  "ob_" +
                  boost::lexical_cast<string>(ob) + 
                  "_cb_" +
                  boost::lexical_cast<string>(cb) + 
                  "_mesh_nr_" +
                  boost::lexical_cast<string>(i + 1) +
                  "_out_predicted.ply",false);
              }
            }

          }
          // ~ write predictively encoded frames
          // print the evaluation results to the output .cs file
          achieved_quality.print_csv_line(compression_arg_ss.str(), res_base_ofstream);
        }
        ////////////// END ASSESMENT //////////////////
      }
      // report convergence and shared macroblock statistics
      double av_macroblock_sharing=0;
      double av_convergence_percentage=0;
      int p_frame_count =0;
      for(int i=0;i<aligned_flags.size();i++)
      {
        if(aligned_flags[i])
        {
          p_frame_count++;
          av_macroblock_sharing += shared_macroblock_percentages[i];
          av_convergence_percentage+= icp_convergence_percentage[i];
        }
      }
      cout << " overall shared macroblock percentage: " << av_macroblock_sharing / p_frame_count <<" total p frames: "  << p_frame_count <<endl;
      cout << " overall shared macroblock convergence percentage: " << av_convergence_percentage / p_frame_count <<" total p frames: "  << p_frame_count <<endl;
      cin.get();
    }
  }
  
  ////////////// END FOR //////////////////	
  return (-1);
}