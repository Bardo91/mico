//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#ifndef _RGBDTOOLS_SEGMENTATION_COLORCLUSTERING_TYPES_BASICTYPES_H_
#define _RGBDTOOLS_SEGMENTATION_COLORCLUSTERING_TYPES_BASICTYPES_H_

namespace rgbd{
	//-----------------------------------------------------------------------------
	/** Basic store types.
	*/
	template<typename Type_>
	struct Vec2{
		// Constructors.
		Vec2():	x(0), y(0)	{};
		Vec2(Type_ _x, Type_ _y):	x(_x),  y(_y)	{};

		// Properties.
		Type_ x, y;		// Coordinates of the point in the image.
	};


	typedef Vec2<double>		Vec2d;
	typedef Vec2<int>			Vec2i;
	typedef Vec2<unsigned>		Vec2ui;
	typedef Vec2<unsigned char> Vec2uc;

	//-----------------------------------------------------------------------------
	/** Basic store types.
	*/
	template<typename Type_>
	struct Vec3{
		// Constructors.
		Vec3():	x(0), y(0), z(0)	{};

		Vec3(Type_ _x, Type_ _y, Type_ _z):	x(_x),  y(_y),	z(_z)	{};

		// Properties.
		Type_ x, y, z;		// Coordinates of the point in the image.
	};


	typedef Vec3<double>		Vec3d;
	typedef Vec3<int>			Vec3i;
	typedef Vec3<unsigned>		Vec3ui;
	typedef Vec3<unsigned char> Vec3uc;

	//-----------------------------------------------------------------------------
	/// Basic class that holds information of generic image object
	class ImageObject{		// Summarized object.
	public:
		/// \brief
		ImageObject(Vec2i _upperLeft, Vec2i _downRight, int _size, int _color){
			mUpperLeft = _upperLeft;
			mDownRight = _downRight;
			mCentroid = Vec2i((_upperLeft.x + _downRight.x)/2, (_upperLeft.y + _downRight.y)/2);
			mWidth = _downRight.x - _upperLeft.x;
			mHeight = _downRight.y - _upperLeft.y;
			mSize = _size;
			mColor = _color;
		};

		/// \brief
		ImageObject(Vec2i _centroid, int _width, int _height, int _size, int _color){
			mCentroid = _centroid;
			mUpperLeft = Vec2i(	mCentroid.x - _width/2,mCentroid.x - _height/2 );
			mDownRight = Vec2i(	mCentroid.x + _width/2,mCentroid.x + _height/2 );
			mWidth = _width;
			mHeight = _height;
			mSize = _size;
			mColor = _color;
		};
			
		/// \brief Join current object with the given one;
		void join(ImageObject _obj) {
			mUpperLeft	= Vec2i(	mUpperLeft.x < _obj.mUpperLeft.x ? mUpperLeft.x : _obj.mUpperLeft.x,
									mUpperLeft.y < _obj.mUpperLeft.y ? mUpperLeft.y : _obj.mUpperLeft.y);
			
			mDownRight	= Vec2i(	mDownRight.x > _obj.mDownRight.x ? mDownRight.x : _obj.mDownRight.x,
									mDownRight.y > _obj.mDownRight.y ? mDownRight.y : _obj.mDownRight.y);

			mCentroid	= Vec2i((mUpperLeft.x + mDownRight.x)/2, (mUpperLeft.y + mDownRight.y)/2);
			mWidth		= mDownRight.x - mUpperLeft.x;
			mHeight		= mDownRight.y - mUpperLeft.y;
			mSize		= mSize + _obj.mSize;
			mColor		= mColor == _obj.mColor ? mColor : -1;
		}

		/// \brief get centroid of object in the image. 666 rename to centroid();
		Vec2i centroid() const {return mCentroid;};
		
		/// \brief get width of object. 666 rename to width().
		int width() const {return mWidth;};
		
		/// \brief get height of object. 666 rename to height().
		int height() const {return mHeight;};
		
		/// \brief get color of object. 666 rename to color(). 666 Only used in CCS, need review.
		int color() const {return mColor;};
		
		/// \brief get number of pixels of the object. 666 rename to size().
		int size() const {return mSize;};
	private:
		Vec2i mCentroid;
		int mWidth, mHeight;
		int mColor;
		int mSize;
		Vec2i mUpperLeft, mDownRight;
	};

}	// nam


#endif	//_RGBDTOOLS_SEGMENTATION_COLORCLUSTERING_TYPES_BASICTYPES_H_