import pygame
import base
import time
#from PIL import Image
import robot_follow_path as car
# import robot_follow_path_2_pid as car
start = (100, 66)
goal = (400, 500)
dimensions = (600, 600)
obsdim = 60
obsnum = 30
iteration_max = 500

def main(obs, graph, map_):
    global start, goal, dimensions, obsdim, obsnum, iteration_max

    # tạo vật cản

    # vẽ vật cản
    map_.drawmap(obs)
    iteration = 0
    t1 = time.time()
    oldcost = float('inf')
    oldpath = None

    n_image = 0
    # # Lưu màn hình Pygame vào file BMP
    # pygame.image.save(map_.map, f"images/screenshot{n_image}.bmp")
    #
    # # Sử dụng Pillow để mở và chuyển file BMP sang PNG hoặc GIF
    # image = Image.open(f"images/screenshot{n_image}.bmp")
    # image.save(f"images/screenshot{n_image}.png")  # Lưu dưới dạng PNG
    # n_image += 1

    while iteration <= iteration_max:
        eslaped = time.time() - t1
        t1 = time.time()
        if eslaped >10:
            raise
        if iteration % 10 == 0:
            n = graph.number_of_nodes()
            x, y, parent = graph.bias(goal)
            pygame.draw.circle(map_.map, map_.red, (x[-1], y[-1]), map_.nodeRadian + 2, map_.nodeThickness)
            pygame.draw.line(map_.map, map_.blue, (x[-1], y[-1]),
                             (x[parent[-1]], y[parent[-1]]), map_.edgeThickness)
            # print(graph.number_of_nodes()-n)
            if graph.number_of_nodes() - n == 1:
                new_node = graph.number_of_nodes() - 1
                neighbors = graph.near_neighbors(new_node, 70)
                graph.rewire(map_.map, new_node, neighbors)

            # time.sleep(0.05)
        else:
            n = graph.number_of_nodes()
            x, y, parent = graph.expand()
            pygame.draw.circle(map_.map, map_.grey, (x[-1], y[-1]), map_.nodeRadian + 2, map_.nodeThickness)
            pygame.draw.line(map_.map, map_.blue, (x[-1], y[-1]),
                             (x[parent[-1]], y[parent[-1]]), map_.edgeThickness)
            if graph.number_of_nodes() - n == 1:
                new_node = graph.number_of_nodes() - 1
                neighbors = graph.near_neighbors(new_node, 70)
                graph.rewire(map_.map, new_node, neighbors)
                # pygame.draw.line(map_.map, map_.blue, (x[-1], y[-1]),
                #                  (x[parent[-1]], y[parent[-1]]), map_.edgeThickness)
            # time.sleep(0.05)
        # time.sleep(0.05)
        pygame.display.update()
        iteration += 1
        # map_.drawpath(graph.getPathcoords())
        if iteration % 100 == 0:
            print(iteration)
        if graph.path_to_goal():
            new_cost = graph.getfinalcost()
            if new_cost < oldcost:
                print("New cost :" + str(new_cost))
                if oldpath:
                    for i in range(0, len(oldpath) - 1):
                        pygame.draw.line(map_.map, (255, 255, 255), oldpath[i], oldpath[i + 1],
                                         map_.edgeThickness + 5)
                        # time.sleep(0.03)
                        pygame.display.update()
                for i in range(0, len(graph.getPathcoords()) - 1):
                    pygame.draw.line(map_.map, (255, 255, 0), graph.getPathcoords()[i], graph.getPathcoords()[i + 1],
                                     map_.edgeThickness + 5)
                    time.sleep(0.03)
                    pygame.display.update()
                oldcost = new_cost
                oldpath = graph.getPathcoords()
                pygame.draw.circle(map_.map, map_.green, map_.start, map_.nodeRadian + 10, 0)
                pygame.draw.circle(map_.map, map_.red, map_.goal, map_.nodeRadian + 20, 1)
        # if iteration % 10 == 0:
        #     # Lưu màn hình Pygame vào file BMP
        #     pygame.image.save(map_.map, f"images/screenshot{n_image}.bmp")
        #
        #     # Sử dụng Pillow để mở và chuyển file BMP sang PNG hoặc GIF
        #     image = Image.open(f"images/screenshot{n_image}.bmp")
        #     image.save(f"images/screenshot{n_image}.png")  # Lưu dưới dạng PNG
        #     # image.save("screenshot.gif")  # Lưu dưới dạng GIF
        #     n_image += 1

    if graph.getfinalcost() != 0:
        print("Optimal cost of " + str(iteration_max) + " iterations is " + str(graph.getfinalcost()))
    else:
        print("don't find any way to goal")
    pygame.display.update()
    # pygame.event.clear()
    # pygame.event.wait(0)


if __name__ == '__main__':

    result = False
    pygame.init()
    map_ = base.RRTmap(start, goal, dimensions, obsdim, obsnum, 'RRT* Path Planning')
    graph_ = base.RRTgraph(start, goal, dimensions, obsdim, obsnum)
    obs_ = graph_.makeObs()  # vật cản đã khuếch đại kích thước để tìm đường tránh va chạm của robot
    i_ = 0
    #print(obs_)
    map_.drawmap(graph_.getTrueObs(obs_, -35))
    pygame.display.update()
    time.sleep(1)
    while not result:
        if i_ > 0:
            graph_.reset()
        try:
            main(obs_, graph_, map_)
            if graph_.goalFlag:
                result = True
            else:
                print("finding path again...")
        except IndexError:
            result = False
        i_ += 1
        if i_ >= 3 and graph_.goalFlag:
            print("size of final path: " + str(len(graph_.getPathcoords())))
            print("final cost is " + str(graph_.cost_2))
            print("final cost is " + str(graph_.getfinalcost()))
            break
        # if i_ > 20:
        #     print("error")
        #     break
        #     #     print("final cost is " + str(graph_.getfinalcost()))
        # #     graph_.reset(all_=True)

    envir = car.Envir(dimensions)
    mobile_car = car.Robot(start, "car_img.png", graph_.waypoints2path())

    running = True
    dt = 0
    lasttime = pygame.time.get_ticks()
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        #     mobile_car.move(event)

        pygame.display.update()
        # envir.map.fill(envir.white)

        # envir.write_info(int(mobile_car.vl), int(mobile_car.vr), mobile_car.theta)

        dt = (pygame.time.get_ticks() - lasttime) / 1000
        lasttime = pygame.time.get_ticks()
        envir.map.fill(envir.white)
        map_.drawmap(graph_.getTrueObs(obs_, -35))
        for coor in graph_.waypoints2path():
            pygame.draw.circle(map_.map, map_.red, coor, map_.nodeRadian, map_.nodeThickness + 3)

        # i = len(graph_.getPathcoords())
        # pid_controller = car.PIDcontroller([car.Robot.x, graph_.y, graph_.theta], [target[0], target[1]])
        mobile_car.move(dt)
        mobile_car.draw(envir.map)
        # mobile_car.runPID(envir.map)
        # for coor in graph_.waypoints2path():
        #     pygame.draw.circle(map_.map, map_.red, coor, map_.nodeRadian, map_.nodeThickness + 3)

        envir.draw_trail((mobile_car.x, mobile_car.y))
        envir.draw_XYaxis_frame((mobile_car.x, mobile_car.y), mobile_car.theta)
