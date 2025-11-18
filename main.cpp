#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

const int CUBE_SIDES = 6;
const int QUAD_V_COUNT = 4;
const int TRI_V_COUNT = 3;
const int LINE_V_COUNT = 2;
const int HCLEN = 4;
const double UNIT_CUBE_QUADS[CUBE_SIDES][QUAD_V_COUNT][HCLEN] = {
    {{0, 0, 0, 1}, {1, 0, 0, 1}, {1, 1, 0, 1}, {0, 1, 0, 1}},
    {{0, 0, 0, 1}, {1, 0, 0, 1}, {1, 0, 1, 1}, {0, 0, 1, 1}},
    {{0, 0, 0, 1}, {0, 0, 1, 1}, {0, 1, 1, 1}, {0, 1, 0, 1}},
    {{0, 1, 0, 1}, {1, 1, 0, 1}, {1, 1, 1, 1}, {0, 1, 1, 1}},
    {{1, 1, 0, 1}, {1, 0, 0, 1}, {1, 0, 1, 1}, {1, 1, 1, 1}},
    {{0, 0, 1, 1}, {0, 1, 1, 1}, {1, 1, 1, 1}, {1, 0, 1, 1}}
};

// Cube Data = CD
const int CD_LENGTH = 6;
const int CD_POS_INFO_LOC = 0;
const int CD_SIZ_INFO_LOC = 3;
void generateCubeData(double cube_data[CD_LENGTH],
                      double cube_verts[CUBE_SIDES][QUAD_V_COUNT][HCLEN]) {
    double pos[HCLEN] = {0, 0, 0, 0};
    double siz[HCLEN] = {1, 1, 1, 1};

    for (int i=0; i<CD_SIZ_INFO_LOC; i++) {
        pos[i] = cube_data[i + CD_POS_INFO_LOC];
        siz[i] = cube_data[i + CD_SIZ_INFO_LOC];
    }

    for (int quadi=0; quadi<CUBE_SIDES; quadi++) {
        for (int verti=0; verti<QUAD_V_COUNT; verti++) {
            for (int ci=0; ci<HCLEN; ci++) {
                // Initialise to 0
                cube_verts[quadi][verti][ci] = 0;

                cube_verts[quadi][verti][ci] = UNIT_CUBE_QUADS[quadi][verti][ci]
                    *siz[ci] + pos[ci];
            }
        }
    }
}

const double CAVALIER_OBLIQUE[HCLEN][HCLEN] = {
    {1, 0, std::pow(2, 0.5)/2.0, 0},
    {0, 1, std::pow(2, 0.5)/2.0, 0},
    {0, 0,                    1, 0},
    {0, 0,                    0, 1}
};

const double STANDARD_ISOMETRIC[HCLEN][HCLEN] = {
    {std::pow(3, 0.5)/2.0, 0, std::pow(3, 0.5)/2.0, 0},
    {            -0.5, 1,              0.5, 0},
    {              -1, 0,                1, 0},
    {               0, 0,                0, 1}
};

void projOntoCoord(const double proj[HCLEN][HCLEN], double (&coord)[HCLEN]) {
    double ncoord[HCLEN] = {0, 0, 0, 0};
    for (int nci=0; nci<HCLEN; nci++) {
        double nc = 0;
        for (int ci=0; ci<HCLEN; ci++) {
            nc += proj[nci][ci]*coord[ci];
        }
        ncoord[nci] = nc;
    }

    for (int nci=0; nci<HCLEN; nci++) {
        coord[nci] = ncoord[nci];
    }
}

void projOntoQuads(const double proj[HCLEN][HCLEN],
                   double (&quads)[][QUAD_V_COUNT][HCLEN],
                   int quadc) {
    for (int quadi=0; quadi<quadc; quadi++) {
        for (int verti=0; verti<QUAD_V_COUNT; verti++) {
            projOntoCoord(proj, quads[quadi][verti]);
        }
    }
}

struct LineSeg4D {
    static constexpr int DIM = 4;
    double dir[DIM];
    double pnt[DIM];
    double min_s;
    double max_s;

    LineSeg4D () {
    }

    LineSeg4D (double sPoint[], double ePoint[], int dim) {
        for (int d=0; d<dim && d<DIM; d++) {
            pnt[d] = sPoint[d];
            dir[d] = ePoint[d]-sPoint[d];
        }
        min_s = 0;
        max_s= 1;
    }

    LineSeg4D (double direction[], double point[], int dim, double mins, double maxs) {
        for (int d=0; d<dim && d<DIM; d++) {
            pnt[d] = point[d];
            dir[d] = direction[d];
        }
        min_s = mins;
        max_s = maxs;
    }

    bool sInRange(double s) {
        return (s > min_s && s < max_s);
    }

    void getPoint(double s, double pout[DIM]) {
        for (int d=0; d<DIM; d++) {
            pout[d] = 0;
            pout[d] = pnt[d] + dir[d]*s;
        }
    }

    void startPoint(double pout[DIM]) {
        getPoint(min_s, pout);
    }

    void endPoint(double pout[DIM]) {
        getPoint(max_s, pout);
    }
};

double int2d2line(LineSeg4D s, LineSeg4D t,
                  bool &sameLine, bool &noIntersect) {
    double disc = 0;
    disc = (s.dir[1]*t.dir[0] - s.dir[0]*t.dir[1]);

    double ss = 0;

    sameLine = false;
    noIntersect = false;

    if (disc == 0) {
        double dp[2] = {0, 0};
        dp[0] = s.pnt[0]-t.pnt[0];
        dp[1] = s.pnt[1]-t.pnt[1];

        if (s.dir[1]*dp[0]-s.dir[0]*dp[1] == 0) {
            sameLine = true;
        } else {
            noIntersect = true;
        }
    } else {
        ss = 1/disc * (t.dir[1]*(s.pnt[0]-t.pnt[0])
                       - t.dir[0]*(s.pnt[1]-t.pnt[1]));

        double st = 0;
        st = 1/disc * (s.dir[1]*(s.pnt[0]-t.pnt[0])
                       - s.dir[0]*(s.pnt[1]-t.pnt[1]));

        if (!(s.sInRange(ss) && t.sInRange(st))) {
            noIntersect = true;
        }
    }
    return ss;
}

double depthxyintri(double tc[TRI_V_COUNT][HCLEN], double point[HCLEN],
                    bool &inTriangle) {
    // Barycentric coords
    double denom=0, s=0, r=0, t=0, z=0;

    denom = (tc[1][1]-tc[2][1])*(tc[0][0]-tc[2][0])
            + (tc[2][0]-tc[1][0])*(tc[0][1]-tc[2][1]);

    s = ((tc[1][1]-tc[2][1])*(point[0]-tc[2][0])
            + (tc[2][0]-tc[1][0])*(point[1]-tc[2][1]))/denom;
    r = ((tc[2][1]-tc[0][1])*(point[0]-tc[2][0])
            + (tc[0][0]-tc[2][0])*(point[1]-tc[2][1]))/denom;
    t = 1-s-r;

    z = s*tc[0][2] + r*tc[1][2] + t*tc[2][2];

    inTriangle = false;
    if (0<=s && s<=1 && 0<=r && r<=1 && 0<=t && t<=1) {
        inTriangle = true;
    }

    return z;
}

//https://stackoverflow.com/questions/27284185/how-does-the-compare-function-in-qsort-work
int compSegS(const void* s1, const void* s2) {
    const double *S1 = (double*)s1, *S2 = (double*)s2;
    return (*S1 > *S2) - (*S1 < *S1);
}


const double MIN_DEPTH_DELTA_OBSTRUCTION = 0.01;
const double MIN_LINE_SEGMENT_SPACING = 0.000001;

int main() {
    int quadc = 0;
    int btric = 0;


    double cv[100][CUBE_SIDES][QUAD_V_COUNT][HCLEN];
    for (int i=0; i<10; i++) {
        for (int j=0; j<10; j++) {
            double cube_data_temp[6] = {i, j, k, 1, 1, 1};
            generateCubeData(cube_data_temp, cv[10*i+j]);
            projOntoQuads(STANDARD_ISOMETRIC, cv[10*i+j], 6);
            quadc += CUBE_SIDES;
        }
    }

    double* pquads = new double[quadc*QUAD_V_COUNT*HCLEN];
    int segmentc = quadc*QUAD_V_COUNT;
    LineSeg4D* segments = new LineSeg4D[segmentc];

    for (int cube=0; cube<100; cube++) {
        for (int face=0; face<CUBE_SIDES; face++) {
            for (int verti=0; verti<QUAD_V_COUNT; verti++) {
                for (int ci=0; ci<HCLEN; ci++) {
                    pquads[cube*CUBE_SIDES*QUAD_V_COUNT*HCLEN + face*QUAD_V_COUNT*HCLEN + verti*HCLEN + ci] = cv[cube][face][verti][ci];
                }
            }
        }
    }

    btric += 2*quadc;
    double* btris = new double[btric*TRI_V_COUNT*HCLEN];
    for (int quadi=0; quadi<quadc; quadi++) {
        // Add line segments
        for (int verti=0; verti<QUAD_V_COUNT; verti++) {
            double sPoint[HCLEN] = {0, 0, 0, 0};
            double ePoint[HCLEN] = {0, 0, 0, 0};

            for (int ci=0; ci<HCLEN; ci++) {
                sPoint[ci] = pquads[quadi*QUAD_V_COUNT*HCLEN + verti*HCLEN + ci];
                ePoint[ci] = pquads[quadi*QUAD_V_COUNT*HCLEN + ((verti+1)%QUAD_V_COUNT)*HCLEN + ci];
            }

            LineSeg4D segment(sPoint, ePoint, 4);
            segments[quadi*QUAD_V_COUNT + verti] = segment;
        }

        // Add blocking triangles
        const int TRIS_PER_QUAD = 2;
        const int TRI_VERTS_PER_QUAD = 6;
        int tris[TRI_VERTS_PER_QUAD] = {0, 1, 2, 2, 3, 0};
        for (int verti=0;verti<TRI_VERTS_PER_QUAD; verti++) {
            for (int ci=0; ci<HCLEN; ci++) {
                // btris[quadi*2][verti][ci] = pquads[quadi][tris[verti]][[ci]
                btris[quadi*TRIS_PER_QUAD*TRI_V_COUNT*HCLEN + verti*HCLEN + ci] =
                    pquads[quadi*QUAD_V_COUNT*HCLEN + tris[verti]*HCLEN + ci];
            }
        }
    }
    //pquads now stores a list of all projected quad data

    int maxsubsegments = segmentc;
    int subsegmentc = 0;
    double* subsegments = new double[maxsubsegments*LINE_V_COUNT*HCLEN];

    const int maxinters = segmentc; // Max number of intersections
    // is the number of other segments
    int interc = 2;
    double* inters = new double[maxinters];
    // At the very minimum endpoints are included

    for (int segmenti=0; segmenti<segmentc; segmenti++) {
        interc = 2;
        inters[0] = 0;
        inters[1] = 1;
        for (int segmentj=0; segmentj<(segmentc-1); segmentj++) {
            if (segmentj >= segmenti) {
                segmentj++;
            }
            bool sameLine = false;
            bool noIntersect = false;
            double s=0;

            s=int2d2line(segments[segmenti], segments[segmentj], sameLine, noIntersect);

            if (!noIntersect && !sameLine) {
                inters[interc] = s;
                interc++;
            }
        }

        // startpoints start from index 0
        int pinteri=0;
        qsort(inters, interc, sizeof(double), compSegS);

        // endpoints start from index 1
        for (int interi=1; interi<interc; interi++) {
            //if (inters[interi] != inters[pinteri]) {//((std::abs(inters[pinteri]) - std::abs(inters[interi])) > MIN_LINE_SEGMENT_SPACING) {
            if (std::abs(inters[pinteri]-inters[interi]) > MIN_LINE_SEGMENT_SPACING) {
                double sPoint[HCLEN] = {0, 0, 0, 0};
                double ePoint[HCLEN] = {0, 0, 0, 0};
                double mPoint[HCLEN] = {0, 0, 0, 0};

                segments[segmenti].getPoint(inters[pinteri], sPoint);
                segments[segmenti].getPoint(inters[interi], ePoint);

                for (int ci=0; ci<HCLEN; ci++) {

                    mPoint[ci] = (sPoint[ci]+ePoint[ci])/2.0;
                }

                bool visible = true;
                for (int btrii=0; btrii<btric; btrii++) {
                    bool inTriangle = false;
                    double depth = 0;

                    double triverts[TRI_V_COUNT][HCLEN];
                    for (int verti=0; verti<TRI_V_COUNT; verti++) {
                        for (int ci=0; ci<HCLEN; ci++) {
                            triverts[verti][ci] = btris[btrii*TRI_V_COUNT*HCLEN + verti*HCLEN + ci];
                        }
                    }

                    depth = depthxyintri(triverts, mPoint, inTriangle);
                    if (inTriangle && (depth > (mPoint[2] + MIN_DEPTH_DELTA_OBSTRUCTION))) {
                        visible = false;
                    }
                }

                if (visible) {
                    if (subsegmentc+1 >= maxsubsegments) {
                        maxsubsegments *= 2;
                        double* tmp = new double[maxsubsegments*LINE_V_COUNT*HCLEN];
                        for (int i=0; i<subsegmentc*LINE_V_COUNT*HCLEN; i++) {
                            tmp[i] = subsegments[i];
                        }
                        delete[] subsegments;
                        subsegments = tmp;
                    }
                    for (int ci=0; ci<HCLEN; ci++) {
                        subsegments[subsegmentc*LINE_V_COUNT*HCLEN + 0*HCLEN + ci] = sPoint[ci];
                        subsegments[subsegmentc*LINE_V_COUNT*HCLEN + 1*HCLEN + ci] = ePoint[ci];
                    }

                    subsegmentc++;
                }
                pinteri = interi;
            }
        }
    }

    std::cout << "\n\n\n";
    for (int subsegmenti=0; subsegmenti<subsegmentc; subsegmenti++) {
        std::cout << "[";
        for (int verti=0; verti<LINE_V_COUNT; verti++) {
            std::cout << "[";
            for (int ci=0; ci<HCLEN; ci++) {
                std::cout << subsegments[subsegmenti*LINE_V_COUNT*HCLEN + verti*HCLEN + ci] << ", ";
            }
            std::cout << "], ";
        }
        std::cout << "], " << std::endl;
    }

    delete[] btris;
    delete[] pquads;
    delete[] segments;
    delete[] subsegments;
    delete[] inters;

    return 0;
}

/*
Sources used at some point during development:
https://stackoverflow.com/questions/17562026/difference-between-passing-an-array-by-value-and-reference-in-c
https://www.geeksforgeeks.org/cpp/structure-vs-class-in-cpp/
https://www.w3schools.com/cpp/cpp_constructors_overloading.asp
https://stackoverflow.com/questions/755835/how-to-add-element-to-c-array
https://www.w3schools.com/cpp/cpp_vectors.asp
https://stackoverflow.com/questions/29734072/error-invalid-use-of-non-static-data-member
https://shendrick.net/Coding%20Tips/2015/03/15/cpparrayvsvector.html
*/
