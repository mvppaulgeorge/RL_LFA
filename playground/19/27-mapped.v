// Benchmark "adder" written by ABC on Wed Jul 17 21:55:48 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n139, new_n140, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n278, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n351,
    new_n353, new_n355, new_n356, new_n358, new_n359, new_n361, new_n362,
    new_n364, new_n365, new_n366, new_n368, new_n369, new_n370;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n04x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n06x4               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor002aa1n12x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand02aa1d08x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand02aa1n03x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nona22aa1n02x4               g008(.a(new_n102), .b(new_n101), .c(new_n103), .out0(new_n104));
  tech160nm_fixnrc02aa1n05x5   g009(.a(\b[3] ), .b(\a[4] ), .out0(new_n105));
  nor022aa1n16x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand42aa1n08x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanb03aa1n12x5               g012(.a(new_n106), .b(new_n107), .c(new_n102), .out0(new_n108));
  nona22aa1n09x5               g013(.a(new_n104), .b(new_n108), .c(new_n105), .out0(new_n109));
  aoi112aa1n03x5               g014(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n110));
  oab012aa1n04x5               g015(.a(new_n110), .b(\a[4] ), .c(\b[3] ), .out0(new_n111));
  nor042aa1n03x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nor022aa1n16x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nona23aa1n03x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  nanp02aa1n04x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nor002aa1d32x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  norb02aa1n06x4               g023(.a(new_n117), .b(new_n118), .out0(new_n119));
  nor002aa1n03x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nand22aa1n04x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  norb02aa1n06x4               g026(.a(new_n121), .b(new_n120), .out0(new_n122));
  nano22aa1n03x7               g027(.a(new_n116), .b(new_n122), .c(new_n119), .out0(new_n123));
  inv000aa1n03x5               g028(.a(new_n123), .o1(new_n124));
  oai022aa1n02x5               g029(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n125));
  nano22aa1n03x7               g030(.a(new_n112), .b(new_n113), .c(new_n121), .out0(new_n126));
  oai122aa1n06x5               g031(.a(new_n117), .b(new_n114), .c(new_n118), .d(\b[7] ), .e(\a[8] ), .o1(new_n127));
  aboi22aa1n06x5               g032(.a(new_n127), .b(new_n126), .c(new_n125), .d(new_n121), .out0(new_n128));
  aoai13aa1n12x5               g033(.a(new_n128), .b(new_n124), .c(new_n109), .d(new_n111), .o1(new_n129));
  xorc02aa1n12x5               g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  aoi012aa1n02x5               g035(.a(new_n100), .b(new_n129), .c(new_n130), .o1(new_n131));
  norb03aa1n03x5               g036(.a(new_n102), .b(new_n101), .c(new_n103), .out0(new_n132));
  oai013aa1n03x5               g037(.a(new_n111), .b(new_n132), .c(new_n108), .d(new_n105), .o1(new_n133));
  inv000aa1d42x5               g038(.a(\a[7] ), .o1(new_n134));
  inv000aa1d42x5               g039(.a(\b[6] ), .o1(new_n135));
  aoai13aa1n02x5               g040(.a(new_n121), .b(new_n120), .c(new_n134), .d(new_n135), .o1(new_n136));
  oaib12aa1n06x5               g041(.a(new_n136), .b(new_n127), .c(new_n126), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n130), .b(new_n137), .c(new_n133), .d(new_n123), .o1(new_n138));
  norb03aa1n02x5               g043(.a(new_n98), .b(new_n97), .c(new_n100), .out0(new_n139));
  nanp02aa1n02x5               g044(.a(new_n138), .b(new_n139), .o1(new_n140));
  oai012aa1n02x5               g045(.a(new_n140), .b(new_n131), .c(new_n99), .o1(\s[10] ));
  nand42aa1n04x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nor042aa1n09x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  nanb03aa1n02x5               g048(.a(new_n143), .b(new_n98), .c(new_n142), .out0(new_n144));
  nanb02aa1n02x5               g049(.a(new_n144), .b(new_n140), .out0(new_n145));
  inv040aa1n02x5               g050(.a(new_n143), .o1(new_n146));
  aoi022aa1n02x5               g051(.a(new_n140), .b(new_n98), .c(new_n146), .d(new_n142), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n145), .b(new_n147), .out0(\s[11] ));
  nor002aa1d24x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nand02aa1n06x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n149), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n143), .b(new_n152), .c(new_n150), .o1(new_n153));
  aoai13aa1n02x5               g058(.a(new_n146), .b(new_n144), .c(new_n138), .d(new_n139), .o1(new_n154));
  aoi022aa1n02x5               g059(.a(new_n145), .b(new_n153), .c(new_n154), .d(new_n151), .o1(\s[12] ));
  nano23aa1n06x5               g060(.a(new_n149), .b(new_n143), .c(new_n150), .d(new_n142), .out0(new_n156));
  and003aa1n02x5               g061(.a(new_n156), .b(new_n130), .c(new_n99), .o(new_n157));
  aoai13aa1n03x5               g062(.a(new_n157), .b(new_n137), .c(new_n133), .d(new_n123), .o1(new_n158));
  nanb03aa1n03x5               g063(.a(new_n149), .b(new_n150), .c(new_n142), .out0(new_n159));
  oai112aa1n03x5               g064(.a(new_n146), .b(new_n98), .c(new_n100), .d(new_n97), .o1(new_n160));
  aoi012aa1n06x5               g065(.a(new_n149), .b(new_n143), .c(new_n150), .o1(new_n161));
  oai012aa1n06x5               g066(.a(new_n161), .b(new_n160), .c(new_n159), .o1(new_n162));
  nanb02aa1n03x5               g067(.a(new_n162), .b(new_n158), .out0(new_n163));
  nor022aa1n08x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nand42aa1n06x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  nano22aa1n03x7               g071(.a(new_n149), .b(new_n142), .c(new_n150), .out0(new_n167));
  tech160nm_fioai012aa1n03p5x5 g072(.a(new_n98), .b(\b[10] ), .c(\a[11] ), .o1(new_n168));
  oab012aa1n04x5               g073(.a(new_n168), .b(new_n97), .c(new_n100), .out0(new_n169));
  inv000aa1n04x5               g074(.a(new_n161), .o1(new_n170));
  aoi112aa1n02x5               g075(.a(new_n170), .b(new_n166), .c(new_n169), .d(new_n167), .o1(new_n171));
  aoi022aa1n02x5               g076(.a(new_n163), .b(new_n166), .c(new_n158), .d(new_n171), .o1(\s[13] ));
  nor022aa1n06x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nand42aa1n08x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  aoi112aa1n02x5               g080(.a(new_n164), .b(new_n175), .c(new_n163), .d(new_n166), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n175), .b(new_n164), .c(new_n163), .d(new_n165), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(\s[14] ));
  nano23aa1n09x5               g083(.a(new_n164), .b(new_n173), .c(new_n174), .d(new_n165), .out0(new_n179));
  aoai13aa1n06x5               g084(.a(new_n179), .b(new_n162), .c(new_n129), .d(new_n157), .o1(new_n180));
  oai012aa1n06x5               g085(.a(new_n174), .b(new_n173), .c(new_n164), .o1(new_n181));
  nor002aa1n16x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nand42aa1d28x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  norb02aa1n03x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  xnbna2aa1n03x5               g089(.a(new_n184), .b(new_n180), .c(new_n181), .out0(\s[15] ));
  inv030aa1n02x5               g090(.a(new_n181), .o1(new_n186));
  aoai13aa1n02x5               g091(.a(new_n184), .b(new_n186), .c(new_n163), .d(new_n179), .o1(new_n187));
  nor042aa1n04x5               g092(.a(\b[15] ), .b(\a[16] ), .o1(new_n188));
  nand42aa1n20x5               g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  norb02aa1n02x5               g094(.a(new_n189), .b(new_n188), .out0(new_n190));
  aoib12aa1n02x5               g095(.a(new_n182), .b(new_n189), .c(new_n188), .out0(new_n191));
  inv000aa1d42x5               g096(.a(new_n182), .o1(new_n192));
  inv000aa1d42x5               g097(.a(new_n184), .o1(new_n193));
  aoai13aa1n02x7               g098(.a(new_n192), .b(new_n193), .c(new_n180), .d(new_n181), .o1(new_n194));
  aoi022aa1n03x5               g099(.a(new_n194), .b(new_n190), .c(new_n187), .d(new_n191), .o1(\s[16] ));
  nano23aa1d15x5               g100(.a(new_n182), .b(new_n188), .c(new_n189), .d(new_n183), .out0(new_n196));
  nand22aa1n06x5               g101(.a(new_n196), .b(new_n179), .o1(new_n197));
  nano32aa1d12x5               g102(.a(new_n197), .b(new_n156), .c(new_n130), .d(new_n99), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n137), .c(new_n133), .d(new_n123), .o1(new_n199));
  aoai13aa1n04x5               g104(.a(new_n196), .b(new_n186), .c(new_n162), .d(new_n179), .o1(new_n200));
  aoi012aa1n02x7               g105(.a(new_n188), .b(new_n182), .c(new_n189), .o1(new_n201));
  nand23aa1n06x5               g106(.a(new_n199), .b(new_n200), .c(new_n201), .o1(new_n202));
  xorc02aa1n12x5               g107(.a(\a[17] ), .b(\b[16] ), .out0(new_n203));
  nano22aa1n02x4               g108(.a(new_n203), .b(new_n200), .c(new_n201), .out0(new_n204));
  aoi022aa1n02x5               g109(.a(new_n204), .b(new_n199), .c(new_n202), .d(new_n203), .o1(\s[17] ));
  nor002aa1d32x5               g110(.a(\b[16] ), .b(\a[17] ), .o1(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n196), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n179), .b(new_n170), .c(new_n169), .d(new_n167), .o1(new_n209));
  aoai13aa1n06x5               g114(.a(new_n201), .b(new_n208), .c(new_n209), .d(new_n181), .o1(new_n210));
  aoai13aa1n03x5               g115(.a(new_n203), .b(new_n210), .c(new_n129), .d(new_n198), .o1(new_n211));
  nor002aa1d32x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  nand42aa1n10x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  norb02aa1n15x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n211), .c(new_n207), .out0(\s[18] ));
  and002aa1n02x5               g120(.a(new_n203), .b(new_n214), .o(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n210), .c(new_n129), .d(new_n198), .o1(new_n217));
  oaoi03aa1n02x5               g122(.a(\a[18] ), .b(\b[17] ), .c(new_n207), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  nor042aa1d18x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  nand22aa1n09x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  norb02aa1n12x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  xnbna2aa1n03x5               g127(.a(new_n222), .b(new_n217), .c(new_n219), .out0(\s[19] ));
  xnrc02aa1n02x5               g128(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g129(.a(new_n222), .b(new_n218), .c(new_n202), .d(new_n216), .o1(new_n225));
  nor042aa1n12x5               g130(.a(\b[19] ), .b(\a[20] ), .o1(new_n226));
  nand02aa1d28x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n227), .b(new_n226), .out0(new_n228));
  inv000aa1d42x5               g133(.a(\a[19] ), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\b[18] ), .o1(new_n230));
  aboi22aa1n03x5               g135(.a(new_n226), .b(new_n227), .c(new_n229), .d(new_n230), .out0(new_n231));
  inv040aa1n02x5               g136(.a(new_n220), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n222), .o1(new_n233));
  aoai13aa1n02x5               g138(.a(new_n232), .b(new_n233), .c(new_n217), .d(new_n219), .o1(new_n234));
  aoi022aa1n03x5               g139(.a(new_n234), .b(new_n228), .c(new_n225), .d(new_n231), .o1(\s[20] ));
  nano32aa1n03x7               g140(.a(new_n233), .b(new_n203), .c(new_n228), .d(new_n214), .out0(new_n236));
  aoai13aa1n03x5               g141(.a(new_n236), .b(new_n210), .c(new_n129), .d(new_n198), .o1(new_n237));
  nanb03aa1n12x5               g142(.a(new_n226), .b(new_n227), .c(new_n221), .out0(new_n238));
  oai112aa1n06x5               g143(.a(new_n232), .b(new_n213), .c(new_n212), .d(new_n206), .o1(new_n239));
  aoi012aa1d18x5               g144(.a(new_n226), .b(new_n220), .c(new_n227), .o1(new_n240));
  oai012aa1d24x5               g145(.a(new_n240), .b(new_n239), .c(new_n238), .o1(new_n241));
  nor042aa1d18x5               g146(.a(\b[20] ), .b(\a[21] ), .o1(new_n242));
  nand42aa1n10x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  norb02aa1n12x5               g148(.a(new_n243), .b(new_n242), .out0(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n241), .c(new_n202), .d(new_n236), .o1(new_n245));
  nano22aa1n03x7               g150(.a(new_n226), .b(new_n221), .c(new_n227), .out0(new_n246));
  tech160nm_fioai012aa1n03p5x5 g151(.a(new_n213), .b(\b[18] ), .c(\a[19] ), .o1(new_n247));
  oab012aa1n04x5               g152(.a(new_n247), .b(new_n206), .c(new_n212), .out0(new_n248));
  inv000aa1n02x5               g153(.a(new_n240), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n249), .b(new_n244), .c(new_n248), .d(new_n246), .o1(new_n250));
  aobi12aa1n03x7               g155(.a(new_n245), .b(new_n250), .c(new_n237), .out0(\s[21] ));
  nor042aa1n06x5               g156(.a(\b[21] ), .b(\a[22] ), .o1(new_n252));
  nand42aa1d28x5               g157(.a(\b[21] ), .b(\a[22] ), .o1(new_n253));
  norb02aa1n02x5               g158(.a(new_n253), .b(new_n252), .out0(new_n254));
  aoib12aa1n02x5               g159(.a(new_n242), .b(new_n253), .c(new_n252), .out0(new_n255));
  inv000aa1d42x5               g160(.a(new_n241), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n242), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n244), .o1(new_n258));
  aoai13aa1n03x5               g163(.a(new_n257), .b(new_n258), .c(new_n237), .d(new_n256), .o1(new_n259));
  aoi022aa1n03x5               g164(.a(new_n259), .b(new_n254), .c(new_n245), .d(new_n255), .o1(\s[22] ));
  inv000aa1n02x5               g165(.a(new_n236), .o1(new_n261));
  nano22aa1n03x7               g166(.a(new_n261), .b(new_n244), .c(new_n254), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n210), .c(new_n129), .d(new_n198), .o1(new_n263));
  nano23aa1d18x5               g168(.a(new_n242), .b(new_n252), .c(new_n253), .d(new_n243), .out0(new_n264));
  aoi012aa1d24x5               g169(.a(new_n252), .b(new_n242), .c(new_n253), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  aoi012aa1n02x5               g171(.a(new_n266), .b(new_n241), .c(new_n264), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n267), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[23] ), .b(\b[22] ), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n268), .c(new_n202), .d(new_n262), .o1(new_n270));
  aoi112aa1n02x5               g175(.a(new_n269), .b(new_n266), .c(new_n241), .d(new_n264), .o1(new_n271));
  aobi12aa1n02x7               g176(.a(new_n270), .b(new_n271), .c(new_n263), .out0(\s[23] ));
  xorc02aa1n03x5               g177(.a(\a[24] ), .b(\b[23] ), .out0(new_n273));
  nor042aa1n09x5               g178(.a(\b[22] ), .b(\a[23] ), .o1(new_n274));
  norp02aa1n02x5               g179(.a(new_n273), .b(new_n274), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n274), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n269), .o1(new_n277));
  aoai13aa1n03x5               g182(.a(new_n276), .b(new_n277), .c(new_n263), .d(new_n267), .o1(new_n278));
  aoi022aa1n02x7               g183(.a(new_n278), .b(new_n273), .c(new_n270), .d(new_n275), .o1(\s[24] ));
  and002aa1n12x5               g184(.a(new_n273), .b(new_n269), .o(new_n280));
  nano22aa1n03x7               g185(.a(new_n261), .b(new_n280), .c(new_n264), .out0(new_n281));
  aoai13aa1n02x7               g186(.a(new_n281), .b(new_n210), .c(new_n129), .d(new_n198), .o1(new_n282));
  aoai13aa1n04x5               g187(.a(new_n264), .b(new_n249), .c(new_n248), .d(new_n246), .o1(new_n283));
  inv000aa1n06x5               g188(.a(new_n280), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[24] ), .b(\b[23] ), .c(new_n276), .carry(new_n285));
  aoai13aa1n12x5               g190(.a(new_n285), .b(new_n284), .c(new_n283), .d(new_n265), .o1(new_n286));
  xorc02aa1n12x5               g191(.a(\a[25] ), .b(\b[24] ), .out0(new_n287));
  aoai13aa1n06x5               g192(.a(new_n287), .b(new_n286), .c(new_n202), .d(new_n281), .o1(new_n288));
  aoai13aa1n06x5               g193(.a(new_n280), .b(new_n266), .c(new_n241), .d(new_n264), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n287), .o1(new_n290));
  and003aa1n02x5               g195(.a(new_n289), .b(new_n290), .c(new_n285), .o(new_n291));
  aobi12aa1n03x7               g196(.a(new_n288), .b(new_n291), .c(new_n282), .out0(\s[25] ));
  tech160nm_fixorc02aa1n03p5x5 g197(.a(\a[26] ), .b(\b[25] ), .out0(new_n293));
  nor042aa1n06x5               g198(.a(\b[24] ), .b(\a[25] ), .o1(new_n294));
  norp02aa1n02x5               g199(.a(new_n293), .b(new_n294), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n286), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n294), .o1(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n290), .c(new_n282), .d(new_n296), .o1(new_n298));
  aoi022aa1n02x7               g203(.a(new_n298), .b(new_n293), .c(new_n288), .d(new_n295), .o1(\s[26] ));
  and002aa1n12x5               g204(.a(new_n293), .b(new_n287), .o(new_n300));
  nano32aa1n03x7               g205(.a(new_n261), .b(new_n300), .c(new_n264), .d(new_n280), .out0(new_n301));
  aoai13aa1n06x5               g206(.a(new_n301), .b(new_n210), .c(new_n129), .d(new_n198), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n300), .o1(new_n303));
  oao003aa1n06x5               g208(.a(\a[26] ), .b(\b[25] ), .c(new_n297), .carry(new_n304));
  aoai13aa1n04x5               g209(.a(new_n304), .b(new_n303), .c(new_n289), .d(new_n285), .o1(new_n305));
  xorc02aa1n12x5               g210(.a(\a[27] ), .b(\b[26] ), .out0(new_n306));
  aoai13aa1n06x5               g211(.a(new_n306), .b(new_n305), .c(new_n202), .d(new_n301), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n304), .o1(new_n308));
  aoi112aa1n02x5               g213(.a(new_n306), .b(new_n308), .c(new_n286), .d(new_n300), .o1(new_n309));
  aobi12aa1n02x7               g214(.a(new_n307), .b(new_n309), .c(new_n302), .out0(\s[27] ));
  xorc02aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .out0(new_n311));
  norp02aa1n02x5               g216(.a(\b[26] ), .b(\a[27] ), .o1(new_n312));
  norp02aa1n02x5               g217(.a(new_n311), .b(new_n312), .o1(new_n313));
  aoi012aa1n09x5               g218(.a(new_n308), .b(new_n286), .c(new_n300), .o1(new_n314));
  inv000aa1n03x5               g219(.a(new_n312), .o1(new_n315));
  inv000aa1d42x5               g220(.a(new_n306), .o1(new_n316));
  aoai13aa1n03x5               g221(.a(new_n315), .b(new_n316), .c(new_n302), .d(new_n314), .o1(new_n317));
  aoi022aa1n03x5               g222(.a(new_n317), .b(new_n311), .c(new_n307), .d(new_n313), .o1(\s[28] ));
  and002aa1n02x5               g223(.a(new_n311), .b(new_n306), .o(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n305), .c(new_n202), .d(new_n301), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .out0(new_n321));
  oao003aa1n02x5               g226(.a(\a[28] ), .b(\b[27] ), .c(new_n315), .carry(new_n322));
  norb02aa1n02x5               g227(.a(new_n322), .b(new_n321), .out0(new_n323));
  inv000aa1d42x5               g228(.a(new_n319), .o1(new_n324));
  aoai13aa1n03x5               g229(.a(new_n322), .b(new_n324), .c(new_n302), .d(new_n314), .o1(new_n325));
  aoi022aa1n03x5               g230(.a(new_n325), .b(new_n321), .c(new_n320), .d(new_n323), .o1(\s[29] ));
  xorb03aa1n02x5               g231(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g232(.a(new_n316), .b(new_n311), .c(new_n321), .out0(new_n328));
  aoai13aa1n02x5               g233(.a(new_n328), .b(new_n305), .c(new_n202), .d(new_n301), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .out0(new_n330));
  inv000aa1d42x5               g235(.a(\a[29] ), .o1(new_n331));
  inv000aa1d42x5               g236(.a(\b[28] ), .o1(new_n332));
  oaib12aa1n02x5               g237(.a(new_n322), .b(\b[28] ), .c(new_n331), .out0(new_n333));
  oaoi13aa1n02x5               g238(.a(new_n330), .b(new_n333), .c(new_n331), .d(new_n332), .o1(new_n334));
  inv000aa1n02x5               g239(.a(new_n328), .o1(new_n335));
  oaib12aa1n02x5               g240(.a(new_n333), .b(new_n332), .c(\a[29] ), .out0(new_n336));
  aoai13aa1n03x5               g241(.a(new_n336), .b(new_n335), .c(new_n302), .d(new_n314), .o1(new_n337));
  aoi022aa1n03x5               g242(.a(new_n337), .b(new_n330), .c(new_n329), .d(new_n334), .o1(\s[30] ));
  nano32aa1n02x4               g243(.a(new_n316), .b(new_n330), .c(new_n311), .d(new_n321), .out0(new_n339));
  aoai13aa1n02x5               g244(.a(new_n339), .b(new_n305), .c(new_n202), .d(new_n301), .o1(new_n340));
  aoi022aa1n02x5               g245(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n341));
  norb02aa1n02x5               g246(.a(\b[30] ), .b(\a[31] ), .out0(new_n342));
  obai22aa1n02x7               g247(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n343));
  aoi112aa1n02x5               g248(.a(new_n343), .b(new_n342), .c(new_n333), .d(new_n341), .o1(new_n344));
  inv000aa1n02x5               g249(.a(new_n339), .o1(new_n345));
  norp02aa1n02x5               g250(.a(\b[29] ), .b(\a[30] ), .o1(new_n346));
  aoi012aa1n02x5               g251(.a(new_n346), .b(new_n333), .c(new_n341), .o1(new_n347));
  aoai13aa1n03x5               g252(.a(new_n347), .b(new_n345), .c(new_n302), .d(new_n314), .o1(new_n348));
  xorc02aa1n02x5               g253(.a(\a[31] ), .b(\b[30] ), .out0(new_n349));
  aoi022aa1n03x5               g254(.a(new_n348), .b(new_n349), .c(new_n344), .d(new_n340), .o1(\s[31] ));
  norb02aa1n02x5               g255(.a(new_n107), .b(new_n106), .out0(new_n351));
  xobna2aa1n03x5               g256(.a(new_n351), .b(new_n104), .c(new_n102), .out0(\s[3] ));
  aoai13aa1n02x5               g257(.a(new_n351), .b(new_n132), .c(\a[2] ), .d(\b[1] ), .o1(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n105), .b(new_n353), .c(new_n107), .out0(\s[4] ));
  nanb02aa1n02x5               g259(.a(new_n114), .b(new_n115), .out0(new_n355));
  inv000aa1d42x5               g260(.a(new_n355), .o1(new_n356));
  xnbna2aa1n03x5               g261(.a(new_n356), .b(new_n109), .c(new_n111), .out0(\s[5] ));
  aoai13aa1n02x5               g262(.a(new_n119), .b(new_n114), .c(new_n133), .d(new_n115), .o1(new_n358));
  aoi112aa1n02x5               g263(.a(new_n114), .b(new_n119), .c(new_n133), .d(new_n356), .o1(new_n359));
  norb02aa1n02x5               g264(.a(new_n358), .b(new_n359), .out0(\s[6] ));
  norb02aa1n02x5               g265(.a(new_n113), .b(new_n112), .out0(new_n361));
  inv000aa1d42x5               g266(.a(new_n118), .o1(new_n362));
  xnbna2aa1n03x5               g267(.a(new_n361), .b(new_n358), .c(new_n362), .out0(\s[7] ));
  aob012aa1n02x5               g268(.a(new_n361), .b(new_n358), .c(new_n362), .out0(new_n364));
  oaib12aa1n02x5               g269(.a(new_n364), .b(\b[6] ), .c(new_n134), .out0(new_n365));
  aboi22aa1n03x5               g270(.a(new_n120), .b(new_n121), .c(new_n134), .d(new_n135), .out0(new_n366));
  aoi022aa1n02x5               g271(.a(new_n365), .b(new_n122), .c(new_n364), .d(new_n366), .o1(\s[8] ));
  nanp02aa1n02x5               g272(.a(new_n133), .b(new_n123), .o1(new_n368));
  nanb02aa1n02x5               g273(.a(new_n130), .b(new_n136), .out0(new_n369));
  aoib12aa1n02x5               g274(.a(new_n369), .b(new_n126), .c(new_n127), .out0(new_n370));
  aoi022aa1n02x5               g275(.a(new_n129), .b(new_n130), .c(new_n368), .d(new_n370), .o1(\s[9] ));
endmodule


