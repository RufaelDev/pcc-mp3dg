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