library(rgl);
library(rstackdeque);
#library(rotations);
library(orientlib);




readTracker <- function(file) {
  recording = read.csv(file)
  #recording %>% select(matches("seq"), starts_with("qua"))
}

plotlyTracker <- function(data) {
  plot_ly(data, x=~seq) %>%
    add_trace(y=~qua.w, name='qua.w', type='scatter', mode='lines') %>%
    add_trace(y=~qua.x, name='qua.x', type='scatter', mode='lines') %>%
    add_trace(y=~qua.y, name='qua.y', type='scatter', mode='lines') %>%
    add_trace(y=~qua.z, name='qua.z', type='scatter', mode='lines')
}

animateTracker <- function(data, start=1, end=0, period=0.2) {
  x = c(1, 0, 0)
  y = c(0, 1, 0)
  z = c(0, 0, 1)
  open3d()
  decorate3d()
  ids = rpqueue()
  if (end == 0) {
    end = length(data$seq)
  }
  for (i in start:end) {
    o = c(i, 0, 0)
    rot = rotmatrix(quaternion(c(data$qua.x[i],data$qua.y[i],data$qua.z[i],data$qua.w[i])))
    par3d(skipRedraw=TRUE)
    while (length(ids) > 3) {
      pop3d(peek_front(ids), type="shapes")
      ids = without_front(ids)
    }
    len0 = length(rgl.ids()$id)
    arrow3d(p0=o, p1=(rot[[1]] %*% x)[,1]+o, type="rotation", col="blue")
    arrow3d(p0=o, p1=(rot[[1]] %*% y)[,1]+o, type="rotation", col="green")
    arrow3d(p0=o, p1=(rot[[1]] %*% z)[,1]+o, type="rotation", col="red")
    par3d(skipRedraw=FALSE)
    arrows = rgl.ids()$id
    ids = insert_back(ids, arrows[(len0+1):length(arrows)])
    Sys.sleep(period)
  }
}

# Plot a position given the position vector, the rotation matrix, and the
# absolute acceleration vector.
addPosition <- function(pos, rot, acc) {
  x = c(0.03, 0, 0)
  y = c(0, 0.1, 0)
  z = c(0, 0, 0.03)
  arrow3d(p0=pos, p1=(rot %*% x)[,1]+pos, type="rotation", col="blue")
  arrow3d(p0=pos, p1=(rot %*% y)[,1]+pos, type="rotation", col="green")
  arrow3d(p0=pos, p1=(rot %*% z)[,1]+pos, type="rotation", col="red")
  #if (all(acc>0)) {
  #  arrow3d(p0=pos, p1=acc+pos, type="rotation", col="yellow")
  #}
}

makeRGL <- function() {
  open3d()
  decorate3d()
  aspect3d("iso")
  #mfrow3d(2,1)
  #layout3d(c(0,1), heights=c(3,1))
}

# liveTracker needs to be preceded by a call to makeRGL(). It connects to esp-link
# and listens to the output of the "just-imu" word to plot stuff.
liveTracker <- function(device) {
  sock = socketConnection(device, 23, open="r", timeout=20)
  on.exit(close(sock))
  #firstScene = subsceneList()[1]
  #useSubscene3d(firstScene)
  quads3d(c(-1,-1,1,1), c(-1,1,1,-1), c(0,0,0,0), col="brown", alpha=0.5)
  lines3d(c(0,0), c(0,0), c(0,1), col="brown")
  o = c(0,0,0)    # position ("origin" of device)
  v = c(0,0,0)    # velocity
  t = 0           # time
  ids = rpqueue()
  count = 0;
  while (TRUE || count < 40) {
    line = readLines(sock, n=1)
    if (length(line) == 0) { next }
    cat("line:", line, "\n")
    if (!startsWith(line, "IMUbuf")) { next }
    d = unlist(strsplit(line, " ", fixed=TRUE))
    qua = unlist(lapply(d[c(9,10,11,8)], as.double))/16384 # x, y, z, w
    acc = unlist(lapply(d[12:14], as.double))/100 # m/s^2

    rot = rotmatrix(quaternion(qua))
    rot = t(rot[[1]])
    ra = (rot %*% acc)[,1]

    t1 = as.double(d[5])
    if (t == 0) {
      t = t1
    } else {
      dt = t1 - t
      #cat(dt, o, v, ra, "\n")
      v = v*0.95 + ra * dt
      o = o*0.95 + v * dt
      t = t1
    }

    #useSubscene3d(firstScene)
    #par3d(skipRedraw=TRUE)
    while (length(ids) > 49) {
      pop3d(peek_front(ids), type="shapes")
      ids = without_front(ids)
    }

    len0 = length(rgl.ids()$id)
    #cat(o, rot, ra)
    addPosition(o, rot, ra)
    arrows = rgl.ids()$id
    ids = insert_back(ids, arrows[(len0+1):length(arrows)])
    #par3d(skipRedraw=FALSE)

    #next3d()
    count = count + 1;
  }
}
liveTracker("esp-link5")

plotTracker <- function(data, start=1, end=0, period=0.2) {
  open3d()
  decorate3d()
  aspect3d("iso")
  if (end == 0) { end = length(data$seq) }
  o = c(0,0,0)
  v = c(0,0,0)
  for (i in start:end) {
    rot = rotmatrix(quaternion(c(data$qua.x[i],data$qua.y[i],data$qua.z[i],data$qua.w[i])))
    acc = c(data$acc.x[i]/1000, data$acc.y[i]/1000, data$acc.z[i]/1000)
    ra = (rot[[1]] %*% acc)[,1]
    addPosition(o, rot, ra)

    v = v + acc
    o = o + v

    Sys.sleep(period)
  }
}
plotTracker(log3, start=2380, end=2500)

testSocket <- function() {
}

