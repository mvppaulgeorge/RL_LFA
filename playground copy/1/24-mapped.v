// Benchmark "adder" written by ABC on Wed Jul 10 17:20:14 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n130, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n145, new_n146, new_n147, new_n149, new_n150,
    new_n151, new_n153, new_n154, new_n155, new_n156, new_n157, new_n158,
    new_n159, new_n161, new_n162, new_n163, new_n164, new_n165, new_n166,
    new_n167, new_n168, new_n170, new_n171, new_n172, new_n173, new_n175,
    new_n176, new_n177, new_n178, new_n179, new_n180, new_n181, new_n182,
    new_n185, new_n186, new_n187, new_n188, new_n189, new_n190, new_n192,
    new_n193, new_n194, new_n195, new_n196, new_n197, new_n198, new_n199,
    new_n200, new_n202, new_n203, new_n204, new_n205, new_n206, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n218, new_n219, new_n220, new_n221, new_n222, new_n224,
    new_n225, new_n226, new_n227, new_n228, new_n229, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n236, new_n237, new_n238, new_n239,
    new_n240, new_n241, new_n243, new_n244, new_n245, new_n246, new_n247,
    new_n248, new_n250, new_n251, new_n252, new_n253, new_n254, new_n255,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n262, new_n263,
    new_n264, new_n265, new_n266, new_n267, new_n268, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n288, new_n291,
    new_n293, new_n295, new_n297;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oai012aa1n02x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n02x4               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  aoi012aa1n02x5               g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n02x4               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  norp03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  orn002aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .o(new_n118));
  oaoi03aa1n02x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  aoi012aa1n02x5               g024(.a(new_n110), .b(new_n112), .c(new_n111), .o1(new_n120));
  oaib12aa1n02x5               g025(.a(new_n120), .b(new_n114), .c(new_n119), .out0(new_n121));
  xorc02aa1n02x5               g026(.a(\a[9] ), .b(\b[8] ), .out0(new_n122));
  aoai13aa1n02x5               g027(.a(new_n122), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n123));
  xnbna2aa1n03x5               g028(.a(new_n97), .b(new_n123), .c(new_n98), .out0(\s[10] ));
  160nm_ficinv00aa1n08x5       g029(.clk(new_n97), .clkout(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n126));
  160nm_ficinv00aa1n08x5       g031(.clk(new_n126), .clkout(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n125), .c(new_n123), .d(new_n98), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norp02aa1n02x5               g036(.a(\b[11] ), .b(\a[12] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[11] ), .b(\a[12] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  aoi112aa1n02x5               g039(.a(new_n134), .b(new_n130), .c(new_n128), .d(new_n131), .o1(new_n135));
  aoai13aa1n02x5               g040(.a(new_n134), .b(new_n130), .c(new_n128), .d(new_n131), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(\s[12] ));
  nano23aa1n02x4               g042(.a(new_n130), .b(new_n132), .c(new_n133), .d(new_n131), .out0(new_n138));
  and003aa1n02x5               g043(.a(new_n138), .b(new_n122), .c(new_n97), .o(new_n139));
  aoai13aa1n02x5               g044(.a(new_n139), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n140));
  160nm_fiao0012aa1n02p5x5     g045(.a(new_n132), .b(new_n130), .c(new_n133), .o(new_n141));
  aoi012aa1n02x5               g046(.a(new_n141), .b(new_n138), .c(new_n126), .o1(new_n142));
  xorc02aa1n02x5               g047(.a(\a[13] ), .b(\b[12] ), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n140), .c(new_n142), .out0(\s[13] ));
  orn002aa1n02x5               g049(.a(\a[13] ), .b(\b[12] ), .o(new_n145));
  aob012aa1n02x5               g050(.a(new_n143), .b(new_n140), .c(new_n142), .out0(new_n146));
  xorc02aa1n02x5               g051(.a(\a[14] ), .b(\b[13] ), .out0(new_n147));
  xnbna2aa1n03x5               g052(.a(new_n147), .b(new_n146), .c(new_n145), .out0(\s[14] ));
  nanp02aa1n02x5               g053(.a(new_n147), .b(new_n143), .o1(new_n149));
  oao003aa1n02x5               g054(.a(\a[14] ), .b(\b[13] ), .c(new_n145), .carry(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n149), .c(new_n140), .d(new_n142), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g057(.a(\b[14] ), .b(\a[15] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[14] ), .b(\a[15] ), .o1(new_n154));
  norp02aa1n02x5               g059(.a(\b[15] ), .b(\a[16] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[15] ), .b(\a[16] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  aoi112aa1n02x5               g062(.a(new_n157), .b(new_n153), .c(new_n151), .d(new_n154), .o1(new_n158));
  aoai13aa1n02x5               g063(.a(new_n157), .b(new_n153), .c(new_n151), .d(new_n154), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(\s[16] ));
  nano23aa1n02x4               g065(.a(new_n153), .b(new_n155), .c(new_n156), .d(new_n154), .out0(new_n161));
  nanp03aa1n02x5               g066(.a(new_n161), .b(new_n143), .c(new_n147), .o1(new_n162));
  nano32aa1n02x4               g067(.a(new_n162), .b(new_n138), .c(new_n122), .d(new_n97), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n164));
  aoi012aa1n02x5               g069(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n165));
  oaib12aa1n02x5               g070(.a(new_n165), .b(new_n150), .c(new_n161), .out0(new_n166));
  oab012aa1n02x4               g071(.a(new_n166), .b(new_n142), .c(new_n162), .out0(new_n167));
  nanp02aa1n02x5               g072(.a(new_n164), .b(new_n167), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g074(.clk(\a[18] ), .clkout(new_n170));
  160nm_ficinv00aa1n08x5       g075(.clk(\a[17] ), .clkout(new_n171));
  160nm_ficinv00aa1n08x5       g076(.clk(\b[16] ), .clkout(new_n172));
  oaoi03aa1n02x5               g077(.a(new_n171), .b(new_n172), .c(new_n168), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[17] ), .c(new_n170), .out0(\s[18] ));
  xroi22aa1d04x5               g079(.a(new_n171), .b(\b[16] ), .c(new_n170), .d(\b[17] ), .out0(new_n175));
  nanp02aa1n02x5               g080(.a(new_n172), .b(new_n171), .o1(new_n176));
  oaoi03aa1n02x5               g081(.a(\a[18] ), .b(\b[17] ), .c(new_n176), .o1(new_n177));
  norp02aa1n02x5               g082(.a(\b[18] ), .b(\a[19] ), .o1(new_n178));
  nanp02aa1n02x5               g083(.a(\b[18] ), .b(\a[19] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  aoai13aa1n02x5               g085(.a(new_n180), .b(new_n177), .c(new_n168), .d(new_n175), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(new_n180), .b(new_n177), .c(new_n168), .d(new_n175), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n181), .b(new_n182), .out0(\s[19] ));
  xnrc02aa1n02x5               g088(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g089(.a(\b[19] ), .b(\a[20] ), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(\b[19] ), .b(\a[20] ), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n186), .b(new_n185), .out0(new_n187));
  nona22aa1n02x4               g092(.a(new_n181), .b(new_n187), .c(new_n178), .out0(new_n188));
  160nm_ficinv00aa1n08x5       g093(.clk(new_n187), .clkout(new_n189));
  oaoi13aa1n02x5               g094(.a(new_n189), .b(new_n181), .c(\a[19] ), .d(\b[18] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n188), .b(new_n190), .out0(\s[20] ));
  nano23aa1n02x4               g096(.a(new_n178), .b(new_n185), .c(new_n186), .d(new_n179), .out0(new_n192));
  nanp02aa1n02x5               g097(.a(new_n175), .b(new_n192), .o1(new_n193));
  oai022aa1n02x5               g098(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n194));
  oaib12aa1n02x5               g099(.a(new_n194), .b(new_n170), .c(\b[17] ), .out0(new_n195));
  nona23aa1n02x4               g100(.a(new_n186), .b(new_n179), .c(new_n178), .d(new_n185), .out0(new_n196));
  aoi012aa1n02x5               g101(.a(new_n185), .b(new_n178), .c(new_n186), .o1(new_n197));
  oai012aa1n02x5               g102(.a(new_n197), .b(new_n196), .c(new_n195), .o1(new_n198));
  160nm_ficinv00aa1n08x5       g103(.clk(new_n198), .clkout(new_n199));
  aoai13aa1n02x5               g104(.a(new_n199), .b(new_n193), .c(new_n164), .d(new_n167), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g106(.a(\b[20] ), .b(\a[21] ), .o1(new_n202));
  xorc02aa1n02x5               g107(.a(\a[21] ), .b(\b[20] ), .out0(new_n203));
  xorc02aa1n02x5               g108(.a(\a[22] ), .b(\b[21] ), .out0(new_n204));
  aoi112aa1n02x5               g109(.a(new_n202), .b(new_n204), .c(new_n200), .d(new_n203), .o1(new_n205));
  aoai13aa1n02x5               g110(.a(new_n204), .b(new_n202), .c(new_n200), .d(new_n203), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g112(.clk(\a[21] ), .clkout(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(\a[22] ), .clkout(new_n209));
  xroi22aa1d04x5               g114(.a(new_n208), .b(\b[20] ), .c(new_n209), .d(\b[21] ), .out0(new_n210));
  nanp03aa1n02x5               g115(.a(new_n210), .b(new_n175), .c(new_n192), .o1(new_n211));
  160nm_ficinv00aa1n08x5       g116(.clk(\b[21] ), .clkout(new_n212));
  oaoi03aa1n02x5               g117(.a(new_n209), .b(new_n212), .c(new_n202), .o1(new_n213));
  160nm_ficinv00aa1n08x5       g118(.clk(new_n213), .clkout(new_n214));
  aoi012aa1n02x5               g119(.a(new_n214), .b(new_n198), .c(new_n210), .o1(new_n215));
  aoai13aa1n02x5               g120(.a(new_n215), .b(new_n211), .c(new_n164), .d(new_n167), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g122(.a(\b[22] ), .b(\a[23] ), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[23] ), .b(\b[22] ), .out0(new_n219));
  xorc02aa1n02x5               g124(.a(\a[24] ), .b(\b[23] ), .out0(new_n220));
  aoi112aa1n02x5               g125(.a(new_n218), .b(new_n220), .c(new_n216), .d(new_n219), .o1(new_n221));
  aoai13aa1n02x5               g126(.a(new_n220), .b(new_n218), .c(new_n216), .d(new_n219), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(\s[24] ));
  and002aa1n02x5               g128(.a(new_n220), .b(new_n219), .o(new_n224));
  160nm_ficinv00aa1n08x5       g129(.clk(new_n224), .clkout(new_n225));
  nano32aa1n02x4               g130(.a(new_n225), .b(new_n210), .c(new_n175), .d(new_n192), .out0(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n197), .clkout(new_n227));
  aoai13aa1n02x5               g132(.a(new_n210), .b(new_n227), .c(new_n192), .d(new_n177), .o1(new_n228));
  aoi112aa1n02x5               g133(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n229));
  oab012aa1n02x4               g134(.a(new_n229), .b(\a[24] ), .c(\b[23] ), .out0(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n225), .c(new_n228), .d(new_n213), .o1(new_n231));
  aoi012aa1n02x5               g136(.a(new_n231), .b(new_n168), .c(new_n226), .o1(new_n232));
  xnrc02aa1n02x5               g137(.a(\b[24] ), .b(\a[25] ), .out0(new_n233));
  160nm_ficinv00aa1n08x5       g138(.clk(new_n233), .clkout(new_n234));
  xnrc02aa1n02x5               g139(.a(new_n232), .b(new_n234), .out0(\s[25] ));
  norp02aa1n02x5               g140(.a(\b[24] ), .b(\a[25] ), .o1(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n236), .clkout(new_n237));
  aoai13aa1n02x5               g142(.a(new_n234), .b(new_n231), .c(new_n168), .d(new_n226), .o1(new_n238));
  xnrc02aa1n02x5               g143(.a(\b[25] ), .b(\a[26] ), .out0(new_n239));
  nanp03aa1n02x5               g144(.a(new_n238), .b(new_n237), .c(new_n239), .o1(new_n240));
  aoi012aa1n02x5               g145(.a(new_n239), .b(new_n238), .c(new_n237), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n240), .b(new_n241), .out0(\s[26] ));
  norp02aa1n02x5               g147(.a(new_n239), .b(new_n233), .o1(new_n243));
  nano22aa1n02x4               g148(.a(new_n211), .b(new_n224), .c(new_n243), .out0(new_n244));
  nanp02aa1n02x5               g149(.a(new_n168), .b(new_n244), .o1(new_n245));
  oao003aa1n02x5               g150(.a(\a[26] ), .b(\b[25] ), .c(new_n237), .carry(new_n246));
  aobi12aa1n02x5               g151(.a(new_n246), .b(new_n231), .c(new_n243), .out0(new_n247));
  xorc02aa1n02x5               g152(.a(\a[27] ), .b(\b[26] ), .out0(new_n248));
  xnbna2aa1n03x5               g153(.a(new_n248), .b(new_n247), .c(new_n245), .out0(\s[27] ));
  norp02aa1n02x5               g154(.a(\b[26] ), .b(\a[27] ), .o1(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n250), .clkout(new_n251));
  aobi12aa1n02x5               g156(.a(new_n248), .b(new_n247), .c(new_n245), .out0(new_n252));
  xnrc02aa1n02x5               g157(.a(\b[27] ), .b(\a[28] ), .out0(new_n253));
  nano22aa1n02x4               g158(.a(new_n252), .b(new_n251), .c(new_n253), .out0(new_n254));
  aobi12aa1n02x5               g159(.a(new_n244), .b(new_n164), .c(new_n167), .out0(new_n255));
  aoai13aa1n02x5               g160(.a(new_n224), .b(new_n214), .c(new_n198), .d(new_n210), .o1(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n243), .clkout(new_n257));
  aoai13aa1n02x5               g162(.a(new_n246), .b(new_n257), .c(new_n256), .d(new_n230), .o1(new_n258));
  oai012aa1n02x5               g163(.a(new_n248), .b(new_n258), .c(new_n255), .o1(new_n259));
  aoi012aa1n02x5               g164(.a(new_n253), .b(new_n259), .c(new_n251), .o1(new_n260));
  norp02aa1n02x5               g165(.a(new_n260), .b(new_n254), .o1(\s[28] ));
  norb02aa1n02x5               g166(.a(new_n248), .b(new_n253), .out0(new_n262));
  aobi12aa1n02x5               g167(.a(new_n262), .b(new_n247), .c(new_n245), .out0(new_n263));
  oao003aa1n02x5               g168(.a(\a[28] ), .b(\b[27] ), .c(new_n251), .carry(new_n264));
  xnrc02aa1n02x5               g169(.a(\b[28] ), .b(\a[29] ), .out0(new_n265));
  nano22aa1n02x4               g170(.a(new_n263), .b(new_n264), .c(new_n265), .out0(new_n266));
  oai012aa1n02x5               g171(.a(new_n262), .b(new_n258), .c(new_n255), .o1(new_n267));
  aoi012aa1n02x5               g172(.a(new_n265), .b(new_n267), .c(new_n264), .o1(new_n268));
  norp02aa1n02x5               g173(.a(new_n268), .b(new_n266), .o1(\s[29] ));
  xorb03aa1n02x5               g174(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g175(.a(new_n248), .b(new_n265), .c(new_n253), .out0(new_n271));
  aobi12aa1n02x5               g176(.a(new_n271), .b(new_n247), .c(new_n245), .out0(new_n272));
  oao003aa1n02x5               g177(.a(\a[29] ), .b(\b[28] ), .c(new_n264), .carry(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[29] ), .b(\a[30] ), .out0(new_n274));
  nano22aa1n02x4               g179(.a(new_n272), .b(new_n273), .c(new_n274), .out0(new_n275));
  oai012aa1n02x5               g180(.a(new_n271), .b(new_n258), .c(new_n255), .o1(new_n276));
  aoi012aa1n02x5               g181(.a(new_n274), .b(new_n276), .c(new_n273), .o1(new_n277));
  norp02aa1n02x5               g182(.a(new_n277), .b(new_n275), .o1(\s[30] ));
  xnrc02aa1n02x5               g183(.a(\b[30] ), .b(\a[31] ), .out0(new_n279));
  norb02aa1n02x5               g184(.a(new_n271), .b(new_n274), .out0(new_n280));
  aobi12aa1n02x5               g185(.a(new_n280), .b(new_n247), .c(new_n245), .out0(new_n281));
  oao003aa1n02x5               g186(.a(\a[30] ), .b(\b[29] ), .c(new_n273), .carry(new_n282));
  nano22aa1n02x4               g187(.a(new_n281), .b(new_n279), .c(new_n282), .out0(new_n283));
  oai012aa1n02x5               g188(.a(new_n280), .b(new_n258), .c(new_n255), .o1(new_n284));
  aoi012aa1n02x5               g189(.a(new_n279), .b(new_n284), .c(new_n282), .o1(new_n285));
  norp02aa1n02x5               g190(.a(new_n285), .b(new_n283), .o1(\s[31] ));
  xnrb03aa1n02x5               g191(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g192(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n288));
  xorb03aa1n02x5               g193(.a(new_n288), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g194(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaib12aa1n02x5               g195(.a(new_n118), .b(new_n116), .c(new_n109), .out0(new_n291));
  xorb03aa1n02x5               g196(.a(new_n291), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoib12aa1n02x5               g197(.a(new_n119), .b(new_n291), .c(new_n115), .out0(new_n293));
  xnrb03aa1n02x5               g198(.a(new_n293), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g199(.a(\a[7] ), .b(\b[6] ), .c(new_n293), .o1(new_n295));
  xorb03aa1n02x5               g200(.a(new_n295), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi112aa1n02x5               g201(.a(new_n121), .b(new_n122), .c(new_n109), .d(new_n117), .o1(new_n297));
  norb02aa1n02x5               g202(.a(new_n123), .b(new_n297), .out0(\s[9] ));
endmodule


