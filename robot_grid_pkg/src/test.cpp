#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

// 그리드 크기 설정
const int ROW = 10;
const int COL = 10;

// 방향 벡터 (상, 하, 좌, 우)
const int dRow[] = {-1, 1, 0, 0};
const int dCol[] = {0, 0, -1, 1};

// 노드 구조체: 좌표, 비용(g), 예상 비용(h)
struct Node {
    int row, col;
    double g, h;

    bool operator<(const Node& other) const {
        return (g + h) > (other.g + other.h);  // A*의 우선순위 큐
    }
};

// 좌표가 유효한지 확인
bool isValid(int row, int col) {
    return (row >= 0 && row < ROW && col >= 0 && col < COL);
}

// 장애물인지 확인
bool isObstacle(int row, int col, const std::vector<std::vector<char>>& grid) {
    return grid[row][col] == '1';
}

// 목적지에 도달했는지 확인
bool isDestination(int row, int col, int destRow, int destCol) {
    return (row == destRow && col == destCol);
}

// 유클리드 거리로 휴리스틱 계산
double calculateH(int row, int col, int destRow, int destCol) {
    return std::sqrt((row - destRow) * (row - destRow) + (col - destCol) * (col - destCol));
}

// 경로 찾기 함수 (A* 알고리즘)
std::vector<std::vector<char>> aStarSearch(std::vector<std::vector<char>>& grid, int startRow, int startCol, int destRow, int destCol) {
    std::priority_queue<Node> openList;
    std::vector<std::vector<bool>> closedList(ROW, std::vector<bool>(COL, false));
    
    grid[startRow][startCol] = '*';  // 시작지점 표시
    grid[destRow][destCol] = '*';    // 도착지점 표시
    
    Node start = {startRow, startCol, 0.0, calculateH(startRow, startCol, destRow, destCol)};
    openList.push(start);

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        int row = current.row;
        int col = current.col;
        closedList[row][col] = true;

        // 목적지에 도착하면 경로 반환
        if (isDestination(row, col, destRow, destCol)) {
            grid[row][col] = '*';  // 도착지 표시
            return grid;
        }

        // 상, 하, 좌, 우로 이동
        for (int i = 0; i < 4; i++) {
            int newRow = row + dRow[i];
            int newCol = col + dCol[i];

            if (isValid(newRow, newCol) && !closedList[newRow][newCol] && !isObstacle(newRow, newCol, grid)) {
                double gNew = current.g + 1.0;
                double hNew = calculateH(newRow, newCol, destRow, destCol);

                openList.push({newRow, newCol, gNew, hNew});
                if (grid[newRow][newCol] != '*') {
                    grid[newRow][newCol] = '+';  // 중간지점 표시
                }
            }
        }
        if (grid[row][col] != '*') {
            grid[row][col] = '0';  // 경로 표시
        }
    }

    std::cout << "경로를 찾을 수 없습니다!" << std::endl;
    return grid;
}

// 그리드 출력 함수
void printGrid(const std::vector<std::vector<char>>& grid) {
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            std::cout << grid[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

// 사용자 입력 함수
void getUserInput(std::vector<std::vector<char>>& grid, int& startRow, int& startCol, int& destRow, int& destCol) {
    int obsRow, obsCol;

    // 장애물 입력받기
    std::cout << "장애물의 좌표를 입력하세요 (x y). 그만하려면 -1 -1을 입력하세요:\n";
    while (true) {
        std::cin >> obsRow >> obsCol;
        if (obsRow == -1 && obsCol == -1) {
            break;
        }
        if (isValid(obsRow, obsCol)) {
            grid[obsRow][obsCol] = '1';  // 장애물 표시
        } else {
            std::cout << "잘못된 좌표입니다. 다시 입력해주세요.\n";
        }
    }

    // 시작점 입력받기
    std::cout << "로봇의 시작 좌표를 입력하세요 (x y): ";
    std::cin >> startRow >> startCol;
    grid[startRow][startCol] = '*';

    // 도착점 입력받기
    std::cout << "로봇의 도착 좌표를 입력하세요 (x y): ";
    std::cin >> destRow >> destCol;
    grid[destRow][destCol] = '*';
}

int main(int argc, char** argv) {
    // 그리드 초기화 ('.' = 빈 공간)
    std::vector<std::vector<char>> grid(ROW, std::vector<char>(COL, '.'));

    int startRow, startCol, destRow, destCol;

    // 사용자 입력 받기
    getUserInput(grid, startRow, startCol, destRow, destCol);

    // A* 알고리즘으로 경로 탐색
    std::vector<std::vector<char>> path = aStarSearch(grid, startRow, startCol, destRow, destCol);

    // 그리드 출력
    printGrid(path);

    return 0;
}
