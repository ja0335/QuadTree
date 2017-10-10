#include "QuadTree.h"
#include <assert.h>
#include <algorithm> 
#include <chrono>


const int MAX_DEPTH = 5;
const int MAX_OBJECTS = 5;

float SquareDistanceToLineSegment(const FVector& Point, const FVector& A, const FVector& B)
{
	FVector AB = B - A;
	FVector AC = Point - A;
	FVector BC = Point - B;

	float E = FVector::DotProduct(AC, AB);
	if (E <= 0)
		return FVector::DotProduct(AC, AC);

	float F = FVector::DotProduct(AB, AB);
	if (E >= F)
		return FVector::DotProduct(BC, BC);

	return std::max(FVector::DotProduct(AC, AC) - (E * E) / F, 0.0f);
}

QuadTree::QuadTree(QuadTree * Parent, int Depth, float X, float Y, float Width, float Height)
{
	this->Parent = Parent;
	this->Depth = Depth;
	this->X = X;
	this->Y = Y;
	this->Width = Width;
	this->Height = Height;

	NorthWest = nullptr;
	NorthEast = nullptr;
	SouthWest = nullptr;
	SouthEast = nullptr;
}

QuadTree::~QuadTree()
{
	delete NorthWest;
	NorthWest = nullptr;

	delete NorthEast;
	NorthEast = nullptr;

	delete SouthWest;
	SouthWest = nullptr;

	delete SouthEast;
	SouthEast = nullptr;

	Lines.empty();
}

bool QuadTree::ContainsPoint(const FVector& Point) const
{
	// If Parent == nullptr then we must do an inclusive check
	bool bInsideAABB = Parent == nullptr ?
		(X <= Point.X && Point.X <= X + Width && Y >= Point.Y && Point.Y >= Y + Height) :
		(X <= Point.X && Point.X <  X + Width && Y >= Point.Y && Point.Y >  Y + Height);

	return bInsideAABB;
}

bool QuadTree::InsertInChilds(const FLine& Line)
{
	bool bResult = false;

	if (NorthWest->Insert(Line))
		bResult = true;

	if (NorthEast->Insert(Line))
		bResult = true;

	if (SouthWest->Insert(Line))
		bResult = true;

	if (SouthEast->Insert(Line))
		bResult = true;

	return bResult;
}

bool QuadTree::Insert(const FLine& Line)
{
	if (DoesRayIntersectsArea(Line.GetStart(), Line.GetEnd() - Line.GetStart()))
	{
		if (NorthWest == nullptr && NorthEast == nullptr && SouthWest == nullptr && SouthEast == nullptr)
		{
			Lines.push_back(Line);

			if (Lines.size() > MAX_OBJECTS && Depth <= MAX_DEPTH)
			{
				Split();

				for (const FLine & tLine : Lines)
				{
					bool bResult = InsertInChilds(tLine);

					// We check bResult because it must be true. If not, there is a bug!
					assert(bResult);
				}

				Lines.empty();
			}

			return true;
		}
		else
		{
			bool bResult = InsertInChilds(Line);

			// We check bResult because it must be true. If not, there is a bug!
			assert(bResult);
		}

		return true;
	}

	return false;
}

void QuadTree::Split()
{
	if (NorthWest == nullptr && NorthEast == nullptr && SouthWest == nullptr && SouthEast == nullptr)
	{
		float NewWidth = Width * 0.5f;
		float NewHeight = Height * 0.5f;

		NorthWest = new QuadTree(this, Depth + 1, X, Y, NewWidth, NewHeight);
		NorthEast = new QuadTree(this, Depth + 1, X + NewWidth, Y, NewWidth, NewHeight);
		SouthWest = new QuadTree(this, Depth + 1, X, Y + NewHeight, NewWidth, NewHeight);
		SouthEast = new QuadTree(this, Depth + 1, X + NewWidth, Y + NewHeight, NewWidth, NewHeight);
	}
}

void QuadTree::Clear()
{
	Lines.empty();

	if (NorthWest != nullptr)
		NorthWest->Clear();
	if (NorthEast != nullptr)
		NorthEast->Clear();
	if (SouthWest != nullptr)
		SouthWest->Clear();
	if (SouthEast != nullptr)
		SouthEast->Clear();
}

bool QuadTree::GetLeafForPoint(const FVector &Point, float &OutX, float &OutY, float &OutWidth, float &OutHeight) const
{
	if (ContainsPoint(Point))
	{
		if (NorthWest != nullptr && NorthEast != nullptr && SouthWest != nullptr && SouthEast != nullptr)
		{
			bool bResult = false;

			if (NorthWest->GetLeafForPoint(Point, OutX, OutY, OutWidth, OutHeight))
				bResult = true;
			else if (NorthEast->GetLeafForPoint(Point, OutX, OutY, OutWidth, OutHeight))
				bResult = true;
			else if (SouthWest->GetLeafForPoint(Point, OutX, OutY, OutWidth, OutHeight))
				bResult = true;
			else if (SouthEast->GetLeafForPoint(Point, OutX, OutY, OutWidth, OutHeight))
				bResult = true;

			// We check bResult because it must be true. If not, there is a bug!
			assert(bResult);

			return true;
		}
		else
		{
			OutX = X;
			OutY = Y;
			OutWidth = Width;
			OutHeight = Height;
			return true;
		}
	}

	return false;
}

/**
* Using the Separating Axis Theorem check if the Quadtree area and the ray collides
*/
bool QuadTree::DoesRayIntersectsArea(const FVector &Position, const FVector &Movement) const
{
	float A = X;
	float B = X + Width;
	float s = (Position.X < Position.X + Movement.X) ? Position.X : Position.X + Movement.X;
	float t = (Position.X < Position.X + Movement.X) ? Position.X + Movement.X : Position.X;

	if (!(s <= B && A <= t))
		return false;

	A = Y ;
	B = Y + Height;
	s = (Position.Y < Position.Y + Movement.Y) ? Position.Y : Position.Y + Movement.Y;
	t = (Position.Y < Position.Y + Movement.Y) ? Position.Y + Movement.Y : Position.Y;

	if (!(s <= B && A <= t))
		return false;

	// Rotate Movement 90 degrees 
	FVector TV(Movement.Y, -Movement.X);

	FVector P0 = FVector(X, Y) - Position;
	FVector P1 = FVector(X + Width, Y) - Position;
	FVector P2 = FVector(X + Width, Y + Height) - Position;
	FVector P3 = FVector(X, Y + Height) - Position;

	float P0_TV = FVector::DotProduct(P0, TV);
	float P1_TV = FVector::DotProduct(P1, TV);
	float P2_TV = FVector::DotProduct(P2, TV);
	float P3_TV = FVector::DotProduct(P3, TV);

	if (P0_TV == 0 || P1_TV == 0 || P2_TV == 0 || P3_TV == 0)
		return true;

	bool bPositiveProjection = false;
	bool bNegativeProjection = false;

	if (P0_TV > 0 || P1_TV > 0 || P2_TV > 0 || P3_TV > 0)
		bPositiveProjection = true;

	if (P0_TV < 0 || P1_TV < 0 || P2_TV < 0 || P3_TV < 0)
		bNegativeProjection = true;

	return bPositiveProjection && bNegativeProjection;
}

bool QuadTree::DoesRectangleIntersectsArea(const FVector &Position, const float &InX, const float &InY, const float &InWidth, const float &InHeight) const
{
	float A = X;
	float B = X + Width;
	float s = InX;
	float t = InX + InWidth;

	if (!(s <= B && A <= t))
		return false;

	A = Y + Height;
	B = Y;
	s = InY + InHeight;
	t = InY;

	if (!(s <= B && A <= t))
		return false;

	return true;
}

void QuadTree::GetRectangleCollidingLeafs(const FVector &Position, const float &InX, const float &InY, const float &InWidth, const float &InHeight, std::vector<const QuadTree*> & Result) const
{
	if (DoesRectangleIntersectsArea(Position, InX, InY, InWidth, InHeight))
	{
		if (NorthWest != nullptr && NorthEast != nullptr && SouthWest != nullptr && SouthEast != nullptr)
		{
			NorthWest->GetRectangleCollidingLeafs(Position, InX, InY, InWidth, InHeight, Result);
			NorthEast->GetRectangleCollidingLeafs(Position, InX, InY, InWidth, InHeight, Result);
			SouthWest->GetRectangleCollidingLeafs(Position, InX, InY, InWidth, InHeight, Result);
			SouthEast->GetRectangleCollidingLeafs(Position, InX, InY, InWidth, InHeight, Result);
		}
		else
		{
			Result.push_back(this);
		}
	}
}

bool QuadTree::TestCircleCollision(const FVector &Position, const float &Radius) const
{
	std::vector<const QuadTree*> Trees;

	GetRectangleCollidingLeafs(Position, Position.X - Radius*0.5f, Position.Y + Radius*0.5f, Radius * 2, Radius * 2, Trees);

	for (const QuadTree * Tree : Trees)
	{
		//Tree->DrawDebugTree();
		const float SquaredRadius = Radius * Radius;

		for (const FLine & Line : Tree->GetLines())
		{
			float SquaredDistance = SquareDistanceToLineSegment(Position, Line.GetStart(), Line.GetEnd());

			if (SquaredDistance < SquaredRadius)
				return true;
		}
	}

	return false;
}

void QuadTree::GetRayCollidingLeafs(const FVector &Start, const FVector &End, std::vector<const QuadTree*> & Result) const
{
	if (DoesRayIntersectsArea(Start, End - Start))
	{
		if (NorthWest != nullptr && NorthEast != nullptr && SouthWest != nullptr && SouthEast != nullptr)
		{
			NorthWest->GetRayCollidingLeafs(Start, End, Result);
			NorthEast->GetRayCollidingLeafs(Start, End, Result);
			SouthWest->GetRayCollidingLeafs(Start, End, Result);
			SouthEast->GetRayCollidingLeafs(Start, End, Result);
		}
		else
		{
			Result.push_back(this);
		}
	}
}

void QuadTree::DrawDebugTree(sf::RenderWindow &window) const
{
	sf::Vertex line1[] = {
		sf::Vertex(sf::Vector2f(X, Y), sf::Color::Red),
		sf::Vertex(sf::Vector2f(X + Width, Y), sf::Color::Red) };
	window.draw(line1, 2, sf::Lines);

	sf::Vertex line2[] = {
		sf::Vertex(sf::Vector2f(X + Width, Y), sf::Color::Red),
		sf::Vertex(sf::Vector2f(X + Width, Y + Height), sf::Color::Red) };
	window.draw(line2, 2, sf::Lines);

	sf::Vertex line3[] = {
		sf::Vertex(sf::Vector2f(X + Width, Y + Height), sf::Color::Red),
		sf::Vertex(sf::Vector2f(X, Y + Height), sf::Color::Red) };
	window.draw(line3, 2, sf::Lines);

	sf::Vertex line4[] = {
		sf::Vertex(sf::Vector2f(X, Y + Height), sf::Color::Red),
		sf::Vertex(sf::Vector2f(X, Y), sf::Color::Red) };
	window.draw(line4, 2, sf::Lines);


	if (NorthWest != nullptr)
		NorthWest->DrawDebugTree(window);
	if (NorthEast != nullptr)
		NorthEast->DrawDebugTree(window);
	if (SouthWest != nullptr)
		SouthWest->DrawDebugTree(window);
	if (SouthEast != nullptr)
		SouthEast->DrawDebugTree(window);
}
