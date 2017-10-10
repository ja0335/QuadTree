#pragma once
#include "SFML/Graphics.hpp"
#include <vector>

class FVector
{
public:
	float X;
	float Y;

	FVector() : X(0), Y(0)
	{
	}

	FVector(float _X, float _Y)
	{
		X = _X;
		Y = _Y;
	}

	static float DotProduct(const FVector& A, const FVector& B)
	{
		return A.X * B.X + A.Y * B.Y;
	}

	static float Cross(const FVector& Vector1, const FVector& Vector2)
	{
		return (Vector1.X * Vector2.Y - Vector1.Y * Vector2.X);
	}

	FVector operator+(const FVector& Other) const 
	{
		return FVector(X + Other.X, Y + Other.Y);
	}

	FVector operator-(const FVector& Other) const 
	{
		return FVector(X - Other.X, Y - Other.Y);
	}

	FVector operator*(float Scalar) const
	{
		return FVector(X * Scalar, Y * Scalar);
	}
};

class FLine
{
public:
	FVector Start;
	FVector End;
	
	FLine(){}

	FLine(const FVector &_Start, const FVector &_End)
	{
		Start = _Start;
		End = _End;
	}

	const FVector GetStart() const
	{
		return Start;
	}

	const FVector GetEnd() const
	{
		return End;
	}
};

class QuadTree
{
public:
	QuadTree(QuadTree * Parent, int Depth, float X, float Y, float Width, float Height);

	~QuadTree();

	bool ContainsPoint(const FVector& Point) const;

	// returns true if insertion was succesfull 
	bool Insert(const FLine& Line);

	void Clear();

	bool GetLeafForPoint(const FVector &Point, float &OutX, float &OutY, float &OutWidth, float &OutHeight) const;

	bool TestCircleCollision(const FVector &Position, const float &Radius) const;
	
	bool DoesRayIntersectSegment(const FVector &Position, const FVector &Movement) const;

	void GetRayCollidingLeafs(const FVector &Start, const FVector &End, std::vector<const QuadTree*> & Result) const;

	const std::vector<FLine> & GetLines() const
	{
		return Lines;
	}

	const QuadTree * GetNorthWest() const
	{
		return NorthWest;
	}

	const QuadTree * GetNorthEast() const
	{
		return NorthEast;
	}

	const QuadTree * GetSouthWest() const
	{
		return SouthWest;
	}

	const QuadTree * GetSouthEast() const
	{
		return SouthEast;
	}

	const float GetX() const
	{
		return X;
	}

	const float GetY() const
	{
		return Y;
	}

	const float GetWidth() const
	{
		return Width;
	}

	const float GetHeight() const
	{
		return Height;
	}

	void DrawDebugTree(sf::RenderWindow &window) const;

private:
	void Split();

	// returns true if insertion was succesfull 
	bool InsertInChilds(const FLine& Line);

	bool DoesRayIntersectsArea(const FVector &Position, const FVector &Movement) const;
	
	bool DoesRectangleIntersectsArea(const FVector &Position, const float &InX, const float &InY, const float &InWidth, const float &InHeight) const;

	void GetRectangleCollidingLeafs(const FVector &Position, const float &InX, const float &InY, const float &InWidth, const float &InHeight, std::vector<const QuadTree*> & Result) const;

private:
	// The depth of a node is the number of edges from the tree's root node to the node.
	int Depth;
	float X;
	float Y;
	float Width;
	float Height;

	// If Parent is null it is the root node
	QuadTree * Parent;

	QuadTree * NorthWest;
	QuadTree * NorthEast;
	QuadTree * SouthWest;
	QuadTree * SouthEast;

	// If Lines is not empty then this is a leaf
	std::vector<FLine> Lines;
};