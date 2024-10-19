import pygame
import base
import time

#from PIL import Image
def main():
    start = (100, 66)
    goal = (660, 500)
    dimensions = (600, 1000)
    obsdim = 50
    obsnum = 50

    pygame.init()
    map_ = base.RRTmap(start, goal, dimensions, obsdim, obsnum)
    graph = base.RRTgraph(start, goal, dimensions, obsdim, obsnum)
    # tạo vật cản
    obstacles = graph.makeObs()
    # vẽ vật cản
    map_.drawmap(obstacles)

    # while(True):
    #     x,y = graph.sample_envir()
    #     n= graph.number_of_nodes()
    #     graph.add_node(n,x,y)
    #     graph.add_edge(n-1,n)
    #
    #     x1,y1 = graph.x[n-1],graph.y[n-1]
    #     x2,y2 = graph.x[n], graph.y[n]
    #     if (graph.isFree()):
    #         pygame.draw.circle(map.map,map.red,(graph.x[n],graph.y[n]),map.nodeRadian,map.nodeThickness)
    #         if not graph.crossObstacle(x1,x2,y1,y2):
    #             # pygame.draw.line(map.map,map.blue,(x1,y1), (x2,y2),map.edgeThickness)
    #             pass
    #     pygame.display.update()
    # # pygame.event.clear()
    # # pygame.event.wait()
    iteration = 0
    t1 = time.time()
    n_image = 0
    # # Lưu màn hình Pygame vào file BMP
    # pygame.image.save(map_.map, f"images/RRT_result/screenshot{n_image}.bmp")
    #
    # # Sử dụng Pillow để mở và chuyển file BMP sang PNG hoặc GIF
    # image = Image.open(f"images/RRT_result/screenshot{n_image}.bmp")
    # image.save(f"images/RRT_result/screenshot{n_image}.png")  # Lưu dưới dạng PNG
    # n_image += 1
    while not graph.path_to_goal():
        eslaped = time.time() - t1
        t1 = time.time()
        if eslaped > 10:
            raise
        if iteration % 10 == 0:
            x, y, parent = graph.bias(goal)

            pygame.draw.circle(map_.map, map_.red, (x[-1], y[-1]), map_.nodeRadian + 2, map_.nodeThickness)
            pygame.draw.line(map_.map, map_.blue, (x[-1], y[-1]),
                             (x[parent[-1]], y[parent[-1]]), map_.edgeThickness)
        else:
            x, y, parent = graph.expand()
            pygame.draw.circle(map_.map, map_.grey, (x[-1], y[-1]), map_.nodeRadian + 2, map_.nodeThickness)
            pygame.draw.line(map_.map, map_.blue, (x[-1], y[-1]),
                             (x[parent[-1]], y[parent[-1]]), map_.edgeThickness)
        pygame.display.update()
        # Lưu màn hình Pygame vào file BMP
        # if iteration % 5 == 0:
        #     pygame.image.save(map_.map, f"images/RRT_result/screenshot{n_image}.bmp")
        #
        #     # Sử dụng Pillow để mở và chuyển file BMP sang PNG hoặc GIF
        #     image = Image.open(f"images/RRT_result/screenshot{n_image}.bmp")
        #     image.save(f"images/RRT_result/screenshot{n_image}.png")  # Lưu dưới dạng PNG
        #     # image.save("screenshot.gif")  # Lưu dưới dạng GIF
        #     n_image += 1
        # time.sleep(0.05)
        iteration += 1
    map_.drawpath(graph.getPathcoords())
    for i in range(0, len(graph.getPathcoords()) - 1):
        pygame.draw.line(map_.map, (0, 115, 200), graph.getPathcoords()[i], graph.getPathcoords()[i + 1],
                         map_.edgeThickness + 5)
        time.sleep(0.03)
        pygame.display.update()
        # pygame.image.save(map_.map, f"images/RRT_result/screenshot{n_image}.bmp")
        #
        # # Sử dụng Pillow để mở và chuyển file BMP sang PNG hoặc GIF
        # image = Image.open(f"images/RRT_result/screenshot{n_image}.bmp")
        # image.save(f"images/RRT_result/screenshot{n_image}.png")  # Lưu dưới dạng PNG
        # # image.save("screenshot.gif")  # Lưu dưới dạng GIF
        # n_image += 1
    # print(graph.cost[-1])
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)


if __name__ == '__main__':
    # result = False
    # while not result:
    #     try:
    #         main()
    #         result = True
    #     except:
    #         result = False
    main()