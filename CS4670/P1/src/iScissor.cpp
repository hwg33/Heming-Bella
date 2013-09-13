/* iScissor.cpp */
/* Main file for implementing project 1.  See TODO statments below
 * (see also correlation.cpp and iScissor.h for additional TODOs) */

#include <assert.h>

#include "correlation.h"
#include "iScissor.h"
#include <cmath>
#include "PriorityQueue.h"
#include <limits>

const double linkLengths[8] = { 1.0, SQRT2, 1.0, SQRT2, 1.0, SQRT2, 1.0, SQRT2 };

// two inlined routines that may help;

inline Node& NODE(Node* n, int i, int j, int width)
{
    return *(n + j * width + i);
}

inline unsigned char PIXEL(const unsigned char* p, int i, int j, int c, int width)
{
    return *(p + 3 * (j * width + i) + c);
}

/************************ TODO 1 ***************************
 *InitNodeBuf
 *	INPUT:
 *		img:	a RGB image of size imgWidth by imgHeight;
 *		nodes:	a allocated buffer of Nodes of the same size, one node corresponds to a pixel in img;
 *  OUPUT:
 *      initializes the column, row, and linkCost fields of each node in the node buffer.
 */

void InitNodeBuf(Node* nodes, const unsigned char* img, int imgWidth, int imgHeight)
{
    double maxD = 0;
    for (int i = 0; i < imgWidth; i++) {
        for (int j = 0; j < imgHeight; j++) {
            Node* node = &NODE(nodes, i, j, imgWidth);
            node->column = i;
            node->row = j;
            for (int k = 0; k < 8; k++) {
                double result[3];
                pixel_filter(result, i, j, img, imgWidth, imgHeight, kernels[k], 3, 3, 1, 0);
                double d = std::sqrt((result[0]*result[0] + result[1]*result[1] + result[2]*result[2]) / 3);
                d = d < 0 ? -d : d;
                if (d > maxD) maxD = d;
                node->linkCost[k] = d;
            }
        }
    }
    printf("max:%f\n", maxD);
    for (int i = 0; i < imgWidth; i++) {
        for (int j = 0; j < imgHeight; j++) {
            Node* node = &NODE(nodes, i, j, imgWidth);
            for (int k = 0; k < 8; k++) {
                double length = k / 2 * 2 == k ? 1 : SQRT2;
                node->linkCost[k] = (maxD - node->linkCost[k]) * length;
            }
        }
    }
}



/************************ END OF TODO 1 ***************************/

static int offsetToLinkIndex(int dx, int dy)
{
    int indices[9] = { 3, 2, 1, 4, -1, 0, 5, 6, 7 };
    int tmp_idx = (dy + 1) * 3 + (dx + 1);
    assert(tmp_idx >= 0 && tmp_idx < 9 && tmp_idx != 4);
    return indices[tmp_idx];
}

/************************ TODO 4 ***************************
 *LiveWireDP:
 *	INPUT:
 *		seedX, seedY:	seed position in nodes
 *		nodes:			node buffer of size width by height;
 *      width, height:  dimensions of the node buffer;
 *		selection:		if selection != NULL, search path only in the subset of nodes[j*width+i] if selection[j*width+i] = 1;
 *						otherwise, search in the whole set of nodes.
 *		numExpanded:		compute the only the first numExpanded number of nodes; (for debugging)
 *	OUTPUT:
 *		computes the minimum path tree from the seed node, by assigning
 *		the prevNode field of each node to its predecessor along the minimum
 *		cost path from the seed to that node.
 */

void LiveWireDP(int seedX, int seedY, Node* nodes, int width, int height, const unsigned char* selection, int numExpanded)
{
    CTypedPtrHeap<Node> pq;
    for (int i = 0; i < width * height; i++) {
        Node* node = &nodes[i];
        if (selection != NULL && selection[i] == 0) node->state = EXPANDED;
        else node->state = INITIAL;
    }
    Node* node = &NODE(nodes, seedX, seedY, width);
    //Node* node = &nodes[seedY * width + seedX];
    node->totalCost = 0;
    node->prevNode = NULL;
    pq.Insert(node);
    int counter = 0;
    while (!pq.IsEmpty() && counter < numExpanded) {
        Node* q = pq.ExtractMin();
        q->state = EXPANDED;
        for (int i = 0; i < 8; i++) {
            int offsetX, offsetY;
            q->nbrNodeOffset(offsetX, offsetY, i);
            //printf("%d, %d\n", (q->row + offsetY) * width + q->column + offsetX, width * height);
            //printf("Row: %d, NewRow: %d, Column: %d, NewColumn: %d, Height: %d, Width: %d\n", q->row, q->row + offsetY, q->column, q->column + offsetX, height, width);
            int newRow = q->row + offsetY, newCol = q->column + offsetX;
            if (newCol < width && newRow < height && newCol >= 0 && newRow >= 0) {
                Node* r = &NODE(nodes, q->column + offsetX, q->row + offsetY, width);
                //Node* r = &nodes[(q->row + offsetY) * width + q->column + offsetX];
                if (r->state != EXPANDED) {
                    if (r->state == INITIAL) {
                        r->totalCost = q->totalCost + q->linkCost[i];
                        r->prevNode = q;
                        r->state = ACTIVE;
                        pq.Insert(r);
                    }
                    else if (r->state == ACTIVE) {
                        if (q->totalCost + q->linkCost[i] < r->totalCost) {
                            r->totalCost = q->totalCost + q->linkCost[i];
                            r->prevNode = q;
                            pq.Update(r);
                        }
                    }
                    //if (counter == 0) {
                        //int sum = 0;
                        //for (int k = 0; k < width * height; k++) sum += nodes[k].state;
                        //printf("sum = %d, %d\n", nodes[(q->row + offsetY) * width + q->column + offsetX].state, r->state);
                    //}
                }
            }
        }
        //printf("%d\n", pq.IsEmpty());
        counter++;
    }
}
/************************ END OF TODO 4 ***************************/

/************************ TODO 5 ***************************
 *MinimumPath:
 *	INPUT:
 *		nodes:				a node buffer of size width by height;
 *		width, height:		dimensions of the node buffer;
 *		freePtX, freePtY:	an input node position;
 *	OUTPUT:
 *		insert a list of nodes along the minimum cost path from the seed node to the input node.
 *		Notice that the seed node in the buffer has a NULL predecessor.
 *		And you want to insert a *pointer* to the Node into path, e.g.,
 *		insert nodes+j*width+i (or &(nodes[j*width+i])) if you want to insert node at (i,j), instead of nodes[nodes+j*width+i]!!!
 *		after the procedure, the seed should be the head of path and the input code should be the tail
 */

void MinimumPath(CTypedPtrDblList <Node>* path, int freePtX, int freePtY, Node* nodes, int width, int height)
{
    Node* currNode = &NODE(nodes, freePtX, freePtY, width);
    CTypedPtrDblElement<Node>* tail = path->AddTail(currNode);
    while (currNode->prevNode != NULL) {
        currNode = currNode->prevNode;
        tail = path->AddPrev(tail, currNode);
    }
}
/************************ END OF TODO 5 ***************************/

/************************ An Extra Credit Item ***************************
 *SeeSnap:
 *	INPUT:
 *		img:				a RGB image buffer of size width by height;
 *		width, height:		dimensions of the image buffer;
 *		x,y:				an input seed position;
 *	OUTPUT:
 *		update the value of x,y to the closest edge based on local image information.
 */

void SeedSnap(int& x, int& y, unsigned char* img, int width, int height)
{
    printf("SeedSnap in iScissor.cpp: to be implemented for extra credit!\n");
}

//generate a cost graph from original image and node buffer with all the link costs;
void MakeCostGraph(unsigned char* costGraph, const Node* nodes, const unsigned char* img, int imgWidth, int imgHeight)
{
    int graphWidth = imgWidth * 3;
    int graphHeight = imgHeight * 3;
    int dgX = 3;
    int dgY = 3 * graphWidth;

    int i, j;
    for (j = 0; j < imgHeight; j++) {
        for (i = 0; i < imgWidth; i++) {
            int nodeIndex = j * imgWidth + i;
            int imgIndex = 3 * nodeIndex;
            int costIndex = 3 * ((3 * j + 1) * graphWidth + (3 * i + 1));

            const Node* node = nodes + nodeIndex;
            const unsigned char* pxl = img + imgIndex;
            unsigned char* cst = costGraph + costIndex;

            cst[0] = pxl[0];
            cst[1] = pxl[1];
            cst[2] = pxl[2];

            //r,g,b channels are grad info in seperate channels;
            DigitizeCost(cst	   + dgX, node->linkCost[0]);
            DigitizeCost(cst - dgY + dgX, node->linkCost[1]);
            DigitizeCost(cst - dgY      , node->linkCost[2]);
            DigitizeCost(cst - dgY - dgX, node->linkCost[3]);
            DigitizeCost(cst	   - dgX, node->linkCost[4]);
            DigitizeCost(cst + dgY - dgX, node->linkCost[5]);
            DigitizeCost(cst + dgY	   ,  node->linkCost[6]);
            DigitizeCost(cst + dgY + dgX, node->linkCost[7]);
        }
    }
}

