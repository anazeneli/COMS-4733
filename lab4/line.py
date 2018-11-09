class Line():
    def __init__(self, arg1, arg2):
        self.ep1 = arg1
        self.ep2 = arg2
        self.m = 0
        self.c = 0
        self.h = 0
        self.v = 0
        ## x values are equal so vertical line
        if arg1[0] == arg2[0]:
            self.type = 'v'
            self.v = arg2[0]
        elif arg1[1] == arg2[1]:
            self.type = 'h'
            self.h = arg2[1]
        else:
            self.type = 's'
            self.m = (arg1[1]-  arg2[1])/(arg1[0] - arg2[0]);
            self.c = arg1[1] - self.m*arg1[0]

    def equality(self, lineCheck):
        if (self.ep1[0] - lineCheck.ep1[0] == 0)&(self.ep1[1] - lineCheck.ep1[1] == 0):
            return True;
        if (self.ep2[0] - lineCheck.ep1[0] == 0)&(self.ep2[1] - lineCheck.ep1[1] == 0):
            return True;

        if (self.ep1[0] - lineCheck.ep2[0] == 0)&(self.ep1[1] - lineCheck.ep2[1] == 0):
            return True;

        if (self.ep2[0] - lineCheck.ep2[0] == 0)&(self.ep2[1] - lineCheck.ep2[1] == 0):
            return True;

        return False;


    def intersect(self, lineCheck):

        # checking if two points are common or not
        if self.equality(lineCheck):
            return False
        # checking if the lines are parallel or not
        if (self.type == lineCheck.type)&(self.m == lineCheck.m):
            return False;


        # taking care of easy cases
        if (self.type == 'h'):
            if (lineCheck.type == 'v'):
                if (min(self.ep1[0],self.ep2[0]) < lineCheck.v)&(max(self.ep1[0],self.ep2[0]) > lineCheck.v):
                    if (min(lineCheck.ep1[1],lineCheck.ep2[1]) < self.h)&(max(lineCheck.ep1[1],lineCheck.ep2[1]) > self.h):
                        return True
                    else:
                        return False
                else:
                    return False
            elif (lineCheck.type == 'h'):
                return False
            else:
                # getting the intersect
                tempX = float((self.h - lineCheck.c)/lineCheck.m)
                if (min(self.ep1[0],self.ep2[0]) < tempX)&(max(self.ep1[0],self.ep2[0]) > tempX):
                    if (min(lineCheck.ep1[1],lineCheck.ep2[1]) < self.h)&(max(lineCheck.ep1[1],lineCheck.ep2[1]) > self.h):
                        return True
                    else:
                        return False
                else:
                    return False

        elif (self.type == 'v'):
            if (lineCheck.type == 'h'):
                if (min(self.ep1[1],self.ep2[1]) < lineCheck.h)&(max(self.ep1[1],self.ep2[1]) > lineCheck.h):
                    if (min(lineCheck.ep1[0],lineCheck.ep2[0]) < self.v)&(max(lineCheck.ep1[0],lineCheck.ep2[0]) > self.v):
                        return True
                    else:
                        return False
                else:
                    return False
            elif (lineCheck.type == 'v'):
                return False
            else:
                tempY = (self.v)*lineCheck.m + lineCheck.c
                if (min(self.ep1[1],self.ep2[1]) < tempY)&(max(self.ep1[1],self.ep2[1]) > tempY):
                    if (min(lineCheck.ep1[0],lineCheck.ep2[0]) < self.v)&(max(lineCheck.ep1[0],lineCheck.ep2[0]) > self.v):
                        return True
                    else:
                        return False
                else:
                    return False

        else:
            if (lineCheck.type == 'h'):
                tempX = float((lineCheck.h - self.c)/self.m)
                if (min(lineCheck.ep1[0],lineCheck.ep2[0]) < tempX)&(max(lineCheck.ep1[0],lineCheck.ep2[0]) > tempX):
                    if (min(self.ep1[1],self.ep2[1]) < lineCheck.h)&(max(self.ep1[1],self.ep2[1]) > lineCheck.h):
                        return True
                    else:
                        return False
                else:
                    return False

            elif (lineCheck.type == 'v'):
                tempY = (lineCheck.v)*self.m + self.c
                if (min(lineCheck.ep1[1],lineCheck.ep2[1]) < tempY)&(max(lineCheck.ep1[1],lineCheck.ep2[1]) > tempY):
                    if (min(self.ep1[0],self.ep2[0]) < lineCheck.v)&(max(self.ep1[0],self.ep2[0]) > lineCheck.v):
                        return True
                    else:
                        return False
                else:
                    return False
            else:
                tempX = (lineCheck.c - self.c)/(self.m - lineCheck.m);
                tempY = (self.m)*tempX + self.c;
                if (min(lineCheck.ep1[1],lineCheck.ep2[1]) < tempY)&(max(lineCheck.ep1[1],lineCheck.ep2[1]) > tempY):
                    if(min(lineCheck.ep1[0],lineCheck.ep2[0]) < tempX)&(max(lineCheck.ep1[0],lineCheck.ep2[0]) > tempX):
                        return True
                    else:
                        return False
                else:
                    return False
