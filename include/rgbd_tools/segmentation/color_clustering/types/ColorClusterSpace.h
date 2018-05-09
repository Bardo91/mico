/////////////////////////////////////////////////////////////////////////////////////////
//	BOVIL: core
//
//		Author: Pablo Ramon Soria
//		Date:	2014-05-03
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <rgbd_tools/segmentation/color_clustering/types/Colors.h>

#include <math.h>
#include <stdint.h>
#include <string>

#ifndef _BOVIL_COLORCLUSTERSPACE_H_
#define _BOVIL_COLORCLUSTERSPACE_H_

#define LOG2 0.3010299957

namespace BOViL{
	/** Class that store a set of clusters in a color space. 777 ad example with image and so on...
	*/
	class ColorClusterSpace {
	public:
		int size;	//	 666 add a method to get size and set private.
		c3u *clusters; //	 666 add a method to get clusters and set private.

		/** \brief
		*/
		ColorClusterSpace(int, unsigned char* _AClass, unsigned char* _BClass, unsigned char* _CClass, const c3u* _colors);
		
		/** \brief
		*/
		~ColorClusterSpace();

	private:
		unsigned char *AClass;
		unsigned char *BClass;
		unsigned char *CClass;

	public:
		/** \brief compute color mermbership.	777 add example.
		*	@param _a: first channel value.
		*	@param _b: second channel value.
		*	@param _c: third channel value.
		*/
		int operator()(unsigned char* _a, unsigned char*_b, unsigned char*_c){
			c3u col(*_a, *_b, *_c);

			int color =  whichColor(col);	
			if(color != -1){
				*_a = clusters[color].a;
				*_b = clusters[color].b;
				*_c = clusters[color].c;
			} else{
				*_a = 0U;
				*_b = 0U;
				*_c = 0U;
			}

			return color;
		};

		int whichColor(c3u& _color){
			int i = (_color.a * (size - 1) / 180); // 666 TODO: improve (gets 5%)
			int j = _color.b*(size - 1) >> 7;
			j = (j>>1) + (j&1);
			int k = _color.c*(size - 1) >> 7;
			k = (k>>1) + (k&1);
			

			int res = AClass[i] & BClass[j] & CClass[k]; //Supposing that colors are not over-layed there's only one possible solution and log2(x) returns an integer /

			int aux = 0;

			if (!res)
				return -1;

			while (!(res & 0x01)) {
				res = res >> 1;
				aux += 1;
			}

			return aux;
		}
	};
} // namespace BOViL.

#endif // _BOVIL_COLORCLUSTERSPACE_H_
