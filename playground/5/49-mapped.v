// Benchmark "adder" written by ABC on Wed Jul 17 14:56:39 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n320,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n346, new_n347, new_n348, new_n349, new_n350, new_n351,
    new_n353, new_n354, new_n355, new_n356, new_n358, new_n360, new_n361,
    new_n363, new_n365, new_n366, new_n368, new_n369;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n16x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  and002aa1n03x5               g003(.a(\b[0] ), .b(\a[1] ), .o(new_n99));
  oaoi03aa1n09x5               g004(.a(\a[2] ), .b(\b[1] ), .c(new_n99), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand22aa1n03x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor042aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand22aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nano23aa1n06x5               g009(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n105));
  tech160nm_fiaoi012aa1n04x5   g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  inv000aa1n02x5               g011(.a(new_n106), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\a[6] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\b[5] ), .o1(new_n109));
  aoi022aa1n02x5               g014(.a(new_n109), .b(new_n108), .c(\a[5] ), .d(\b[4] ), .o1(new_n110));
  norp02aa1n02x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  aoi012aa1n02x5               g016(.a(new_n111), .b(\a[6] ), .c(\b[5] ), .o1(new_n112));
  aoi022aa1d24x5               g017(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\a[7] ), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\b[6] ), .o1(new_n115));
  nand42aa1n02x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  oai112aa1n02x5               g021(.a(new_n113), .b(new_n116), .c(\b[7] ), .d(\a[8] ), .o1(new_n117));
  nano22aa1n03x7               g022(.a(new_n117), .b(new_n110), .c(new_n112), .out0(new_n118));
  aoai13aa1n09x5               g023(.a(new_n118), .b(new_n107), .c(new_n105), .d(new_n100), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nor002aa1n02x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  aoai13aa1n02x5               g026(.a(new_n120), .b(new_n121), .c(new_n115), .d(new_n114), .o1(new_n122));
  nand42aa1n03x5               g027(.a(new_n109), .b(new_n108), .o1(new_n123));
  nanp02aa1n12x5               g028(.a(\b[5] ), .b(\a[6] ), .o1(new_n124));
  oai112aa1n02x5               g029(.a(new_n123), .b(new_n124), .c(\b[4] ), .d(\a[5] ), .o1(new_n125));
  oai122aa1n03x5               g030(.a(new_n124), .b(\a[8] ), .c(\b[7] ), .d(\a[7] ), .e(\b[6] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n113), .b(new_n126), .out0(new_n127));
  aobi12aa1n06x5               g032(.a(new_n122), .b(new_n127), .c(new_n125), .out0(new_n128));
  nand02aa1d06x5               g033(.a(new_n119), .b(new_n128), .o1(new_n129));
  tech160nm_fixnrc02aa1n05x5   g034(.a(\b[8] ), .b(\a[9] ), .out0(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(new_n129), .b(new_n131), .o1(new_n132));
  nor042aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nand42aa1n04x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  nanb02aa1n02x5               g039(.a(new_n133), .b(new_n134), .out0(new_n135));
  xobna2aa1n03x5               g040(.a(new_n135), .b(new_n132), .c(new_n98), .out0(\s[10] ));
  norp02aa1n02x5               g041(.a(new_n133), .b(new_n97), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n130), .c(new_n119), .d(new_n128), .o1(new_n138));
  tech160nm_finand02aa1n03p5x5 g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nor022aa1n12x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nano22aa1n02x4               g045(.a(new_n140), .b(new_n134), .c(new_n139), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(new_n138), .b(new_n141), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n140), .o1(new_n143));
  aoi022aa1n02x5               g048(.a(new_n138), .b(new_n134), .c(new_n143), .d(new_n139), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n142), .b(new_n144), .out0(\s[11] ));
  nor002aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand42aa1n06x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  xnbna2aa1n03x5               g053(.a(new_n148), .b(new_n142), .c(new_n143), .out0(\s[12] ));
  nona23aa1n02x5               g054(.a(new_n139), .b(new_n147), .c(new_n146), .d(new_n140), .out0(new_n150));
  nor003aa1n03x5               g055(.a(new_n150), .b(new_n135), .c(new_n130), .o1(new_n151));
  nano22aa1n03x5               g056(.a(new_n146), .b(new_n139), .c(new_n147), .out0(new_n152));
  tech160nm_fioai012aa1n03p5x5 g057(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .o1(new_n153));
  oab012aa1n04x5               g058(.a(new_n153), .b(new_n97), .c(new_n133), .out0(new_n154));
  aoi012aa1n03x5               g059(.a(new_n146), .b(new_n140), .c(new_n147), .o1(new_n155));
  aob012aa1n02x5               g060(.a(new_n155), .b(new_n154), .c(new_n152), .out0(new_n156));
  nor042aa1n06x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  aoai13aa1n06x5               g064(.a(new_n159), .b(new_n156), .c(new_n129), .d(new_n151), .o1(new_n160));
  inv000aa1n03x5               g065(.a(new_n155), .o1(new_n161));
  aoi112aa1n02x5               g066(.a(new_n161), .b(new_n159), .c(new_n154), .d(new_n152), .o1(new_n162));
  aobi12aa1n02x5               g067(.a(new_n162), .b(new_n129), .c(new_n151), .out0(new_n163));
  norb02aa1n02x5               g068(.a(new_n160), .b(new_n163), .out0(\s[13] ));
  inv000aa1d42x5               g069(.a(new_n157), .o1(new_n165));
  nor042aa1n02x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nand42aa1n02x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  xnbna2aa1n03x5               g073(.a(new_n168), .b(new_n160), .c(new_n165), .out0(\s[14] ));
  nano23aa1n09x5               g074(.a(new_n157), .b(new_n166), .c(new_n167), .d(new_n158), .out0(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n156), .c(new_n129), .d(new_n151), .o1(new_n171));
  tech160nm_fioai012aa1n03p5x5 g076(.a(new_n167), .b(new_n166), .c(new_n157), .o1(new_n172));
  nor022aa1n12x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nand42aa1n03x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  nanb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n171), .c(new_n172), .out0(\s[15] ));
  aob012aa1n02x5               g082(.a(new_n176), .b(new_n171), .c(new_n172), .out0(new_n178));
  nor022aa1n04x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanp02aa1n04x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nanb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(new_n181));
  aoib12aa1n02x5               g086(.a(new_n173), .b(new_n180), .c(new_n179), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n173), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n175), .c(new_n171), .d(new_n172), .o1(new_n184));
  aboi22aa1n03x5               g089(.a(new_n181), .b(new_n184), .c(new_n178), .d(new_n182), .out0(\s[16] ));
  nor042aa1n06x5               g090(.a(new_n130), .b(new_n135), .o1(new_n186));
  nona23aa1d24x5               g091(.a(new_n180), .b(new_n174), .c(new_n173), .d(new_n179), .out0(new_n187));
  nona23aa1n09x5               g092(.a(new_n170), .b(new_n186), .c(new_n187), .d(new_n150), .out0(new_n188));
  inv040aa1n03x5               g093(.a(new_n188), .o1(new_n189));
  nand02aa1d06x5               g094(.a(new_n129), .b(new_n189), .o1(new_n190));
  inv000aa1d42x5               g095(.a(new_n187), .o1(new_n191));
  aoai13aa1n04x5               g096(.a(new_n170), .b(new_n161), .c(new_n154), .d(new_n152), .o1(new_n192));
  aob012aa1n03x5               g097(.a(new_n191), .b(new_n192), .c(new_n172), .out0(new_n193));
  tech160nm_fiaoi012aa1n03p5x5 g098(.a(new_n179), .b(new_n173), .c(new_n180), .o1(new_n194));
  nanp03aa1d12x5               g099(.a(new_n190), .b(new_n193), .c(new_n194), .o1(new_n195));
  xorc02aa1n02x5               g100(.a(\a[17] ), .b(\b[16] ), .out0(new_n196));
  nano22aa1n02x4               g101(.a(new_n196), .b(new_n193), .c(new_n194), .out0(new_n197));
  aoi022aa1n02x5               g102(.a(new_n197), .b(new_n190), .c(new_n195), .d(new_n196), .o1(\s[17] ));
  nor042aa1n09x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  aoi012aa1d18x5               g105(.a(new_n188), .b(new_n119), .c(new_n128), .o1(new_n201));
  aoai13aa1n12x5               g106(.a(new_n194), .b(new_n187), .c(new_n192), .d(new_n172), .o1(new_n202));
  oaih12aa1n02x5               g107(.a(new_n196), .b(new_n202), .c(new_n201), .o1(new_n203));
  nor042aa1n03x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nand22aa1n04x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  norb02aa1n06x4               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  xnbna2aa1n03x5               g111(.a(new_n206), .b(new_n203), .c(new_n200), .out0(\s[18] ));
  and002aa1n02x5               g112(.a(new_n196), .b(new_n206), .o(new_n208));
  oai012aa1n06x5               g113(.a(new_n208), .b(new_n202), .c(new_n201), .o1(new_n209));
  oaoi03aa1n02x5               g114(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  nor002aa1n20x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nanp02aa1n04x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  norb02aa1n12x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  xnbna2aa1n03x5               g119(.a(new_n214), .b(new_n209), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n06x5               g121(.a(new_n214), .b(new_n210), .c(new_n195), .d(new_n208), .o1(new_n217));
  nor042aa1n04x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand02aa1d06x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  inv000aa1d42x5               g125(.a(\a[19] ), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\b[18] ), .o1(new_n222));
  aboi22aa1n03x5               g127(.a(new_n218), .b(new_n219), .c(new_n221), .d(new_n222), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n212), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n214), .o1(new_n225));
  aoai13aa1n02x5               g130(.a(new_n224), .b(new_n225), .c(new_n209), .d(new_n211), .o1(new_n226));
  aoi022aa1n03x5               g131(.a(new_n226), .b(new_n220), .c(new_n217), .d(new_n223), .o1(\s[20] ));
  nano32aa1n03x7               g132(.a(new_n225), .b(new_n196), .c(new_n220), .d(new_n206), .out0(new_n228));
  oai012aa1n06x5               g133(.a(new_n228), .b(new_n202), .c(new_n201), .o1(new_n229));
  nanb03aa1n12x5               g134(.a(new_n218), .b(new_n219), .c(new_n213), .out0(new_n230));
  oai112aa1n06x5               g135(.a(new_n224), .b(new_n205), .c(new_n204), .d(new_n199), .o1(new_n231));
  aoi012aa1n12x5               g136(.a(new_n218), .b(new_n212), .c(new_n219), .o1(new_n232));
  oai012aa1n18x5               g137(.a(new_n232), .b(new_n231), .c(new_n230), .o1(new_n233));
  inv000aa1d42x5               g138(.a(new_n233), .o1(new_n234));
  nanp02aa1n06x5               g139(.a(new_n229), .b(new_n234), .o1(new_n235));
  xnrc02aa1n12x5               g140(.a(\b[20] ), .b(\a[21] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  nano22aa1n09x5               g142(.a(new_n218), .b(new_n213), .c(new_n219), .out0(new_n238));
  oai012aa1n02x5               g143(.a(new_n205), .b(\b[18] ), .c(\a[19] ), .o1(new_n239));
  oab012aa1n06x5               g144(.a(new_n239), .b(new_n199), .c(new_n204), .out0(new_n240));
  inv030aa1n02x5               g145(.a(new_n232), .o1(new_n241));
  aoi112aa1n02x5               g146(.a(new_n237), .b(new_n241), .c(new_n240), .d(new_n238), .o1(new_n242));
  aoi022aa1n02x5               g147(.a(new_n235), .b(new_n237), .c(new_n229), .d(new_n242), .o1(\s[21] ));
  nand02aa1n02x5               g148(.a(new_n235), .b(new_n237), .o1(new_n244));
  xnrc02aa1n12x5               g149(.a(\b[21] ), .b(\a[22] ), .out0(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  nor042aa1n06x5               g151(.a(\b[20] ), .b(\a[21] ), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n245), .b(new_n247), .out0(new_n248));
  inv000aa1d42x5               g153(.a(new_n247), .o1(new_n249));
  aoai13aa1n02x7               g154(.a(new_n249), .b(new_n236), .c(new_n229), .d(new_n234), .o1(new_n250));
  aoi022aa1n03x5               g155(.a(new_n250), .b(new_n246), .c(new_n244), .d(new_n248), .o1(\s[22] ));
  nor042aa1n06x5               g156(.a(new_n245), .b(new_n236), .o1(new_n252));
  and002aa1n02x5               g157(.a(new_n228), .b(new_n252), .o(new_n253));
  oai012aa1n06x5               g158(.a(new_n253), .b(new_n202), .c(new_n201), .o1(new_n254));
  oao003aa1n12x5               g159(.a(\a[22] ), .b(\b[21] ), .c(new_n249), .carry(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  aoi012aa1n02x5               g161(.a(new_n256), .b(new_n233), .c(new_n252), .o1(new_n257));
  nanp02aa1n06x5               g162(.a(new_n254), .b(new_n257), .o1(new_n258));
  xorc02aa1n12x5               g163(.a(\a[23] ), .b(\b[22] ), .out0(new_n259));
  aoi112aa1n02x5               g164(.a(new_n259), .b(new_n256), .c(new_n233), .d(new_n252), .o1(new_n260));
  aoi022aa1n02x5               g165(.a(new_n258), .b(new_n259), .c(new_n254), .d(new_n260), .o1(\s[23] ));
  nand22aa1n03x5               g166(.a(new_n258), .b(new_n259), .o1(new_n262));
  xorc02aa1n02x5               g167(.a(\a[24] ), .b(\b[23] ), .out0(new_n263));
  nor042aa1n06x5               g168(.a(\b[22] ), .b(\a[23] ), .o1(new_n264));
  norp02aa1n02x5               g169(.a(new_n263), .b(new_n264), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n264), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n259), .o1(new_n267));
  aoai13aa1n02x7               g172(.a(new_n266), .b(new_n267), .c(new_n254), .d(new_n257), .o1(new_n268));
  aoi022aa1n03x5               g173(.a(new_n268), .b(new_n263), .c(new_n262), .d(new_n265), .o1(\s[24] ));
  inv000aa1n02x5               g174(.a(new_n228), .o1(new_n270));
  and002aa1n03x5               g175(.a(new_n263), .b(new_n259), .o(new_n271));
  nano22aa1n02x4               g176(.a(new_n270), .b(new_n271), .c(new_n252), .out0(new_n272));
  oai012aa1n06x5               g177(.a(new_n272), .b(new_n202), .c(new_n201), .o1(new_n273));
  aoai13aa1n06x5               g178(.a(new_n252), .b(new_n241), .c(new_n240), .d(new_n238), .o1(new_n274));
  inv020aa1n03x5               g179(.a(new_n271), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[24] ), .b(\b[23] ), .c(new_n266), .carry(new_n276));
  aoai13aa1n12x5               g181(.a(new_n276), .b(new_n275), .c(new_n274), .d(new_n255), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n277), .o1(new_n278));
  nanp02aa1n06x5               g183(.a(new_n273), .b(new_n278), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n271), .b(new_n256), .c(new_n233), .d(new_n252), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n280), .o1(new_n282));
  and003aa1n02x5               g187(.a(new_n281), .b(new_n282), .c(new_n276), .o(new_n283));
  aoi022aa1n02x5               g188(.a(new_n279), .b(new_n280), .c(new_n273), .d(new_n283), .o1(\s[25] ));
  nand02aa1n02x5               g189(.a(new_n279), .b(new_n280), .o1(new_n285));
  xorc02aa1n02x5               g190(.a(\a[26] ), .b(\b[25] ), .out0(new_n286));
  nor042aa1n03x5               g191(.a(\b[24] ), .b(\a[25] ), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n286), .b(new_n287), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n287), .o1(new_n289));
  aoai13aa1n02x7               g194(.a(new_n289), .b(new_n282), .c(new_n273), .d(new_n278), .o1(new_n290));
  aoi022aa1n03x5               g195(.a(new_n290), .b(new_n286), .c(new_n285), .d(new_n288), .o1(\s[26] ));
  and002aa1n18x5               g196(.a(new_n286), .b(new_n280), .o(new_n292));
  nano32aa1n03x7               g197(.a(new_n270), .b(new_n292), .c(new_n252), .d(new_n271), .out0(new_n293));
  oai012aa1n12x5               g198(.a(new_n293), .b(new_n202), .c(new_n201), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[26] ), .b(\b[25] ), .c(new_n289), .carry(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  aoi012aa1n09x5               g201(.a(new_n296), .b(new_n277), .c(new_n292), .o1(new_n297));
  nanp02aa1n02x5               g202(.a(new_n297), .b(new_n294), .o1(new_n298));
  xorc02aa1n12x5               g203(.a(\a[27] ), .b(\b[26] ), .out0(new_n299));
  aoi112aa1n02x5               g204(.a(new_n299), .b(new_n296), .c(new_n277), .d(new_n292), .o1(new_n300));
  aoi022aa1n02x5               g205(.a(new_n298), .b(new_n299), .c(new_n294), .d(new_n300), .o1(\s[27] ));
  inv000aa1d42x5               g206(.a(new_n292), .o1(new_n302));
  aoai13aa1n04x5               g207(.a(new_n295), .b(new_n302), .c(new_n281), .d(new_n276), .o1(new_n303));
  aoai13aa1n02x5               g208(.a(new_n299), .b(new_n303), .c(new_n195), .d(new_n293), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[28] ), .b(\b[27] ), .out0(new_n305));
  norp02aa1n02x5               g210(.a(\b[26] ), .b(\a[27] ), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n305), .b(new_n306), .o1(new_n307));
  inv000aa1n03x5               g212(.a(new_n306), .o1(new_n308));
  inv000aa1d42x5               g213(.a(new_n299), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n308), .b(new_n309), .c(new_n297), .d(new_n294), .o1(new_n310));
  aoi022aa1n03x5               g215(.a(new_n310), .b(new_n305), .c(new_n304), .d(new_n307), .o1(\s[28] ));
  and002aa1n02x5               g216(.a(new_n305), .b(new_n299), .o(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n303), .c(new_n195), .d(new_n293), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n312), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .c(new_n308), .carry(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n314), .c(new_n297), .d(new_n294), .o1(new_n316));
  xorc02aa1n02x5               g221(.a(\a[29] ), .b(\b[28] ), .out0(new_n317));
  norb02aa1n02x5               g222(.a(new_n315), .b(new_n317), .out0(new_n318));
  aoi022aa1n03x5               g223(.a(new_n316), .b(new_n317), .c(new_n313), .d(new_n318), .o1(\s[29] ));
  inv000aa1d42x5               g224(.a(\a[2] ), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n99), .b(\b[1] ), .c(new_n320), .out0(\s[2] ));
  nano22aa1n12x5               g226(.a(new_n309), .b(new_n305), .c(new_n317), .out0(new_n322));
  aoai13aa1n02x5               g227(.a(new_n322), .b(new_n303), .c(new_n195), .d(new_n293), .o1(new_n323));
  inv000aa1d42x5               g228(.a(new_n322), .o1(new_n324));
  inv000aa1d42x5               g229(.a(\b[28] ), .o1(new_n325));
  inv000aa1d42x5               g230(.a(\a[29] ), .o1(new_n326));
  oaib12aa1n02x5               g231(.a(new_n315), .b(\b[28] ), .c(new_n326), .out0(new_n327));
  oaib12aa1n02x5               g232(.a(new_n327), .b(new_n325), .c(\a[29] ), .out0(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n324), .c(new_n297), .d(new_n294), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .out0(new_n330));
  oaoi13aa1n02x5               g235(.a(new_n330), .b(new_n327), .c(new_n326), .d(new_n325), .o1(new_n331));
  aoi022aa1n03x5               g236(.a(new_n329), .b(new_n330), .c(new_n323), .d(new_n331), .o1(\s[30] ));
  nanb02aa1n02x5               g237(.a(\b[30] ), .b(\a[31] ), .out0(new_n333));
  nanb02aa1n02x5               g238(.a(\a[31] ), .b(\b[30] ), .out0(new_n334));
  nanp02aa1n02x5               g239(.a(new_n334), .b(new_n333), .o1(new_n335));
  nano32aa1n06x5               g240(.a(new_n309), .b(new_n330), .c(new_n305), .d(new_n317), .out0(new_n336));
  aoai13aa1n02x5               g241(.a(new_n336), .b(new_n303), .c(new_n195), .d(new_n293), .o1(new_n337));
  inv000aa1d42x5               g242(.a(new_n336), .o1(new_n338));
  norp02aa1n02x5               g243(.a(\b[29] ), .b(\a[30] ), .o1(new_n339));
  aoi022aa1n02x5               g244(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n340));
  aoi012aa1n02x5               g245(.a(new_n339), .b(new_n327), .c(new_n340), .o1(new_n341));
  aoai13aa1n03x5               g246(.a(new_n341), .b(new_n338), .c(new_n297), .d(new_n294), .o1(new_n342));
  oai112aa1n02x5               g247(.a(new_n333), .b(new_n334), .c(\b[29] ), .d(\a[30] ), .o1(new_n343));
  aoi012aa1n02x5               g248(.a(new_n343), .b(new_n327), .c(new_n340), .o1(new_n344));
  aoi022aa1n03x5               g249(.a(new_n342), .b(new_n335), .c(new_n337), .d(new_n344), .o1(\s[31] ));
  inv000aa1d42x5               g250(.a(\b[1] ), .o1(new_n346));
  aoi022aa1n02x5               g251(.a(new_n346), .b(new_n320), .c(\a[1] ), .d(\b[0] ), .o1(new_n347));
  oaib12aa1n02x5               g252(.a(new_n347), .b(new_n346), .c(\a[2] ), .out0(new_n348));
  nanb02aa1n02x5               g253(.a(new_n103), .b(new_n104), .out0(new_n349));
  inv000aa1d42x5               g254(.a(new_n349), .o1(new_n350));
  aboi22aa1n03x5               g255(.a(new_n103), .b(new_n104), .c(new_n320), .d(new_n346), .out0(new_n351));
  aoi022aa1n02x5               g256(.a(new_n351), .b(new_n348), .c(new_n350), .d(new_n100), .o1(\s[3] ));
  nanp02aa1n02x5               g257(.a(new_n105), .b(new_n100), .o1(new_n353));
  nanp02aa1n02x5               g258(.a(new_n353), .b(new_n106), .o1(new_n354));
  obai22aa1n02x7               g259(.a(new_n102), .b(new_n101), .c(\a[3] ), .d(\b[2] ), .out0(new_n355));
  aoi012aa1n02x5               g260(.a(new_n355), .b(new_n350), .c(new_n100), .o1(new_n356));
  oaoi13aa1n02x5               g261(.a(new_n356), .b(new_n354), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorc02aa1n02x5               g262(.a(\a[5] ), .b(\b[4] ), .out0(new_n358));
  xnbna2aa1n03x5               g263(.a(new_n358), .b(new_n353), .c(new_n106), .out0(\s[5] ));
  aoi012aa1n02x5               g264(.a(new_n111), .b(new_n354), .c(new_n358), .o1(new_n360));
  tech160nm_fiao0012aa1n02p5x5 g265(.a(new_n125), .b(new_n354), .c(new_n358), .o(new_n361));
  aoai13aa1n02x5               g266(.a(new_n361), .b(new_n360), .c(new_n123), .d(new_n124), .o1(\s[6] ));
  xorc02aa1n02x5               g267(.a(\a[7] ), .b(\b[6] ), .out0(new_n363));
  xobna2aa1n03x5               g268(.a(new_n363), .b(new_n361), .c(new_n124), .out0(\s[7] ));
  norb02aa1n02x5               g269(.a(new_n120), .b(new_n121), .out0(new_n365));
  nanp03aa1n02x5               g270(.a(new_n361), .b(new_n124), .c(new_n363), .o1(new_n366));
  xnbna2aa1n03x5               g271(.a(new_n365), .b(new_n366), .c(new_n116), .out0(\s[8] ));
  nanp02aa1n02x5               g272(.a(new_n130), .b(new_n122), .o1(new_n368));
  aoi012aa1n02x5               g273(.a(new_n368), .b(new_n127), .c(new_n125), .o1(new_n369));
  aoi022aa1n02x5               g274(.a(new_n129), .b(new_n131), .c(new_n119), .d(new_n369), .o1(\s[9] ));
endmodule


