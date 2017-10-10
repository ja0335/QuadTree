#include "SFML/Graphics.hpp"
#include "SFML/System.hpp"
#include "QuadTree.h"

#include <chrono>
#include <iostream>


using namespace std;

float RandomFloat(float a, float b) {
	float random = ((float)rand()) / (float)RAND_MAX;
	float diff = b - a;
	float r = random * diff;
	return a + r;
}


bool LineLineIntersection(const FVector& StartLineSegment1, const FVector& EndLineSegment1, const FVector& StartLineSegment2, const FVector & EndLineSegment2)
{
	FVector LineVector1 = EndLineSegment1 - StartLineSegment1;
	FVector LineVector2 = EndLineSegment2 - StartLineSegment2;
	float LineVectorCross = FVector::Cross(LineVector1, LineVector2);
	float LineToLineVectorCross = FVector::Cross(StartLineSegment2 - StartLineSegment1, LineVector1);

	// If LineVector1 x LineVector2 = 0 and (StartLineSegment2 - StartLineSegment1) x LineVector1 = 0, then the two lines are collinear.
	if (LineVectorCross == 0 && LineToLineVectorCross == 0)
		return false;

	// 3. If LineVector1 x LineVector2 = 0 and (StartLineSegment2 - StartLineSegment1) x LineVector1 != 0, then the two lines are parallel and non-intersecting.
	if (LineVectorCross == 0 && LineToLineVectorCross != 0)
		return false;

	// t = (StartLineSegment2 - StartLineSegment1) x LineVector2 / (LineVector1 x LineVector2)
	float t = FVector::Cross(StartLineSegment2 - StartLineSegment1, LineVector2) / LineVectorCross;

	// u = (StartLineSegment2 - StartLineSegment1) x LineVector1 / (LineVector1 x LineVector2)
	float u = FVector::Cross(StartLineSegment2 - StartLineSegment1, LineVector1) / LineVectorCross;

	// 4. If LineVector1 x LineVector2 != 0 and 0 <= t <= 1 and 0 <= u <= 1
	// the two line segments meet at the point StartLineSegment1 + t LineVector1 = StartLineSegment2 + u LineVector2.
	if (LineVectorCross != 0 && (t >= 0 && t <= 1) && (u >= 0 && u <= 1))
	{
		// An intersection was found.
		return true;
	}

	// 5. Otherwise, the two line segments are not parallel but do not intersect.
	return false;
}

int main(int argc, char *argv[])
{
	for (int i{ 0 }; i < argc; ++i)
		cout << argv[i] << endl;

	int WindowWidth = 1280;
	int WindowHeight = 720;

	std::vector<FLine> Lines;

	QuadTree * Tree = new QuadTree(nullptr, 0, 0, 0, WindowWidth, WindowHeight);

	FVector Offset = FVector(WindowWidth, WindowHeight) * 0.25f;

	float Divisions = 2;
	for (int i{ 1 }; i < WindowWidth / 4; i += 1)
	//for (int i{ 1 }; i < 50; i += 10)
	{
		float Perimeter = 2.0f * 3.1415926535f * float(i);
		float Angle = (2.0f * 3.1415926535f) / Divisions;

		for (int j{ 0 }; j < Divisions; j += 2)
		{
			FVector Start = FVector(i * cos(j * Angle), i * sin(j * Angle)) + Offset;
			FVector End = FVector(i * cos((j * Angle) + Angle), i * sin((j * Angle) + Angle)) + Offset;
			FLine Line = FLine(Start, End);

			Lines.push_back(Line);
			Tree->Insert(Line);
		}

		Divisions = RandomFloat(Divisions, Divisions+10);
	}

	sf::RenderWindow window(sf::VideoMode(WindowWidth, WindowHeight), "My window");

	bool bUseFastCollision = true;
	bool bPause = false;

	// run the program as long as the window is open
	while (window.isOpen())
	{
		// check all the window's events that were triggered since the last iteration of the loop
		sf::Event event;
		while (window.pollEvent(event))
		{
			// "close requested" event: we close the window
			if (event.type == sf::Event::Closed || sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
				window.close();

			if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
				bUseFastCollision = !bUseFastCollision;

			if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
				bPause = !bPause;
		}
		
		if (bPause)
			continue;

		// clear the window with black color
		window.clear(sf::Color::Black);

		// draw everything here...
		for (int i{ 0 }; i < Lines.size(); i++)
		{
			sf::Vertex line[] =
			{
				sf::Vertex(sf::Vector2f(Lines[i].GetStart().X, Lines[i].GetStart().Y)),
				sf::Vertex(sf::Vector2f(Lines[i].GetEnd().X, Lines[i].GetEnd().Y))
			};

			window.draw(line, 2, sf::Lines);
		}
				
		sf::Vector2i LocalPosition = sf::Mouse::getPosition(window);
		sf::Vector2i LineStart(LocalPosition - sf::Vector2i(5, 5));
		sf::Vector2i LineEnd(LocalPosition + sf::Vector2i(5, -5));

		FVector FLineStart = FVector(LineStart.x, LineStart.y);
		FVector FLineEnd = FVector(LineEnd.x, LineEnd.y);
		FLine TheLine = FLine(FLineStart, FLineEnd);

		sf::Color LineColor = sf::Color::Yellow;

		auto Start = std::chrono::steady_clock::now();

		if (bUseFastCollision)
		{
			std::vector<const QuadTree*> ResultTree;
			Tree->GetRayCollidingLeafs(FVector(LineStart.x, LineStart.y), FVector(LineEnd.x, LineEnd.y), ResultTree);

			for (const QuadTree * tTree : ResultTree)
			{
				for (const FLine TLine : tTree->GetLines())
				{
					if (LineLineIntersection(TheLine.GetStart(), TheLine.GetEnd(), TLine.GetStart(), TLine.GetEnd()))
					{
						LineColor = sf::Color::Magenta;
						break;
					}
				}
			}
			auto End = std::chrono::steady_clock::now();
			auto ElapsedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(End - Start).count();
			cout << "Elapsed time ONE: " << ElapsedTime << endl;
		}
		else
		{
			for (int i{ 0 }; i < Lines.size(); i++)
			{
				if (LineLineIntersection(TheLine.GetStart(), TheLine.GetEnd(), Lines[i].GetStart(), Lines[i].GetEnd()))
				{
					LineColor = sf::Color::Magenta;
					break;
				}
			}
			auto End = std::chrono::steady_clock::now();
			auto ElapsedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(End - Start).count();
			cout << "Elapsed time TWO: " << ElapsedTime << endl;
		}


		sf::Vertex line[] =
		{
			sf::Vertex(sf::Vector2f(LineStart.x, LineStart.y), LineColor),
			sf::Vertex(sf::Vector2f(LineEnd.x, LineEnd.y), LineColor)
		};

		window.draw(line, 2, sf::Lines);

		Tree->DrawDebugTree(window);

		// end the current frame
		window.display();
	}


	return EXIT_SUCCESS;
}