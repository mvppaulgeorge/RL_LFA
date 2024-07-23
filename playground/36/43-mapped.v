// Benchmark "adder" written by ABC on Thu Jul 18 06:48:21 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n149, new_n150, new_n151, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n158, new_n159, new_n160, new_n161,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n174, new_n175, new_n176, new_n177,
    new_n178, new_n179, new_n180, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n187, new_n188, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n196, new_n197, new_n198, new_n199, new_n200,
    new_n201, new_n202, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n219, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n227, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n239, new_n240,
    new_n241, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n260, new_n261, new_n262, new_n263,
    new_n264, new_n265, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n274, new_n275, new_n276, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n284, new_n285, new_n286,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n302,
    new_n303, new_n304, new_n305, new_n306, new_n307, new_n308, new_n309,
    new_n311, new_n312, new_n313, new_n314, new_n315, new_n316, new_n317,
    new_n318, new_n320, new_n321, new_n322, new_n323, new_n324, new_n325,
    new_n326, new_n327, new_n328, new_n330, new_n331, new_n332, new_n333,
    new_n334, new_n335, new_n336, new_n339, new_n340, new_n341, new_n342,
    new_n343, new_n344, new_n345, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n353, new_n354, new_n357, new_n358, new_n361,
    new_n363, new_n364, new_n366;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n09x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand02aa1d24x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n09x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nand02aa1d16x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  inv000aa1d42x5               g008(.a(\b[3] ), .o1(new_n104));
  nanb02aa1n12x5               g009(.a(\a[4] ), .b(new_n104), .out0(new_n105));
  nand42aa1n04x5               g010(.a(new_n105), .b(new_n103), .o1(new_n106));
  inv040aa1d32x5               g011(.a(\a[3] ), .o1(new_n107));
  inv040aa1d32x5               g012(.a(\b[2] ), .o1(new_n108));
  nand02aa1n03x5               g013(.a(new_n108), .b(new_n107), .o1(new_n109));
  nand22aa1n06x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nand02aa1n03x5               g015(.a(new_n109), .b(new_n110), .o1(new_n111));
  nor042aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nor042aa1n04x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  aoi012aa1n06x5               g018(.a(new_n112), .b(new_n113), .c(new_n103), .o1(new_n114));
  oai013aa1n09x5               g019(.a(new_n114), .b(new_n102), .c(new_n106), .d(new_n111), .o1(new_n115));
  nor002aa1d32x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  nand22aa1n12x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nand22aa1n12x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nor002aa1d32x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nona23aa1n09x5               g024(.a(new_n118), .b(new_n117), .c(new_n119), .d(new_n116), .out0(new_n120));
  nand42aa1d28x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  nor002aa1d32x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  nanb02aa1n06x5               g027(.a(new_n122), .b(new_n121), .out0(new_n123));
  inv040aa1d32x5               g028(.a(\a[5] ), .o1(new_n124));
  inv040aa1d28x5               g029(.a(\b[4] ), .o1(new_n125));
  nand02aa1d06x5               g030(.a(new_n125), .b(new_n124), .o1(new_n126));
  nanp02aa1n04x5               g031(.a(\b[4] ), .b(\a[5] ), .o1(new_n127));
  nand02aa1d04x5               g032(.a(new_n126), .b(new_n127), .o1(new_n128));
  nor043aa1n06x5               g033(.a(new_n120), .b(new_n123), .c(new_n128), .o1(new_n129));
  aoai13aa1n12x5               g034(.a(new_n121), .b(new_n122), .c(new_n125), .d(new_n124), .o1(new_n130));
  ao0012aa1n03x7               g035(.a(new_n116), .b(new_n119), .c(new_n117), .o(new_n131));
  oabi12aa1n06x5               g036(.a(new_n131), .b(new_n120), .c(new_n130), .out0(new_n132));
  nand42aa1n08x5               g037(.a(\b[8] ), .b(\a[9] ), .o1(new_n133));
  norb02aa1n06x5               g038(.a(new_n133), .b(new_n97), .out0(new_n134));
  aoai13aa1n06x5               g039(.a(new_n134), .b(new_n132), .c(new_n115), .d(new_n129), .o1(new_n135));
  nor022aa1n16x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  nand42aa1n06x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  norb02aa1n03x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n135), .c(new_n98), .out0(\s[10] ));
  inv000aa1n02x5               g044(.a(new_n99), .o1(new_n140));
  aob012aa1n02x5               g045(.a(new_n140), .b(new_n100), .c(new_n101), .out0(new_n141));
  norb02aa1n02x5               g046(.a(new_n103), .b(new_n112), .out0(new_n142));
  norb02aa1n03x5               g047(.a(new_n110), .b(new_n113), .out0(new_n143));
  nanp03aa1n02x5               g048(.a(new_n141), .b(new_n142), .c(new_n143), .o1(new_n144));
  nano23aa1n06x5               g049(.a(new_n119), .b(new_n116), .c(new_n117), .d(new_n118), .out0(new_n145));
  nona22aa1n02x4               g050(.a(new_n145), .b(new_n123), .c(new_n128), .out0(new_n146));
  inv000aa1n09x5               g051(.a(new_n130), .o1(new_n147));
  aoi012aa1d18x5               g052(.a(new_n131), .b(new_n145), .c(new_n147), .o1(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n146), .c(new_n144), .d(new_n114), .o1(new_n149));
  oai022aa1n02x5               g054(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n137), .b(new_n150), .c(new_n149), .d(new_n134), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n16x5               g057(.a(\b[10] ), .b(\a[11] ), .o1(new_n153));
  xnrc02aa1n12x5               g058(.a(\b[10] ), .b(\a[11] ), .out0(new_n154));
  inv040aa1n02x5               g059(.a(new_n150), .o1(new_n155));
  aoi122aa1n06x5               g060(.a(new_n154), .b(\a[10] ), .c(\b[9] ), .d(new_n135), .e(new_n155), .o1(new_n156));
  nor002aa1d24x5               g061(.a(\b[11] ), .b(\a[12] ), .o1(new_n157));
  nanp02aa1n04x5               g062(.a(\b[11] ), .b(\a[12] ), .o1(new_n158));
  nanb02aa1n09x5               g063(.a(new_n157), .b(new_n158), .out0(new_n159));
  oab012aa1n03x5               g064(.a(new_n159), .b(new_n156), .c(new_n153), .out0(new_n160));
  norb03aa1n02x5               g065(.a(new_n159), .b(new_n156), .c(new_n153), .out0(new_n161));
  nor002aa1n02x5               g066(.a(new_n160), .b(new_n161), .o1(\s[12] ));
  nano23aa1n03x5               g067(.a(new_n159), .b(new_n154), .c(new_n138), .d(new_n134), .out0(new_n163));
  aoai13aa1n06x5               g068(.a(new_n163), .b(new_n132), .c(new_n115), .d(new_n129), .o1(new_n164));
  aoi022aa1d24x5               g069(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n165));
  oai012aa1d24x5               g070(.a(new_n165), .b(new_n136), .c(new_n97), .o1(new_n166));
  nor042aa1n04x5               g071(.a(new_n157), .b(new_n153), .o1(new_n167));
  aoi022aa1d24x5               g072(.a(new_n166), .b(new_n167), .c(\b[11] ), .d(\a[12] ), .o1(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  nor002aa1d32x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  nand42aa1d28x5               g075(.a(\b[12] ), .b(\a[13] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n164), .c(new_n169), .out0(\s[13] ));
  norp03aa1n03x5               g078(.a(new_n102), .b(new_n106), .c(new_n111), .o1(new_n174));
  inv020aa1n02x5               g079(.a(new_n114), .o1(new_n175));
  tech160nm_fioai012aa1n04x5   g080(.a(new_n129), .b(new_n174), .c(new_n175), .o1(new_n176));
  nano23aa1n02x4               g081(.a(new_n97), .b(new_n136), .c(new_n137), .d(new_n133), .out0(new_n177));
  nona22aa1n03x5               g082(.a(new_n177), .b(new_n154), .c(new_n159), .out0(new_n178));
  aoai13aa1n03x5               g083(.a(new_n169), .b(new_n178), .c(new_n176), .d(new_n148), .o1(new_n179));
  aoi012aa1n03x5               g084(.a(new_n170), .b(new_n179), .c(new_n171), .o1(new_n180));
  xnrb03aa1n03x5               g085(.a(new_n180), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d32x5               g086(.a(\b[13] ), .b(\a[14] ), .o1(new_n182));
  nand42aa1n10x5               g087(.a(\b[13] ), .b(\a[14] ), .o1(new_n183));
  nona23aa1n09x5               g088(.a(new_n183), .b(new_n171), .c(new_n170), .d(new_n182), .out0(new_n184));
  inv000aa1n03x5               g089(.a(new_n170), .o1(new_n185));
  oaoi03aa1n02x5               g090(.a(\a[14] ), .b(\b[13] ), .c(new_n185), .o1(new_n186));
  inv000aa1n02x5               g091(.a(new_n186), .o1(new_n187));
  aoai13aa1n06x5               g092(.a(new_n187), .b(new_n184), .c(new_n164), .d(new_n169), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n06x5               g094(.a(\b[14] ), .b(\a[15] ), .o1(new_n190));
  inv000aa1n02x5               g095(.a(new_n190), .o1(new_n191));
  nano23aa1n03x5               g096(.a(new_n170), .b(new_n182), .c(new_n183), .d(new_n171), .out0(new_n192));
  nand02aa1d10x5               g097(.a(\b[14] ), .b(\a[15] ), .o1(new_n193));
  nanb02aa1n02x5               g098(.a(new_n190), .b(new_n193), .out0(new_n194));
  inv000aa1d42x5               g099(.a(new_n194), .o1(new_n195));
  aoai13aa1n03x5               g100(.a(new_n195), .b(new_n186), .c(new_n179), .d(new_n192), .o1(new_n196));
  nor002aa1n20x5               g101(.a(\b[15] ), .b(\a[16] ), .o1(new_n197));
  nand02aa1d28x5               g102(.a(\b[15] ), .b(\a[16] ), .o1(new_n198));
  norb02aa1d27x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  tech160nm_fiaoi012aa1n02p5x5 g105(.a(new_n200), .b(new_n196), .c(new_n191), .o1(new_n201));
  aoi112aa1n02x5               g106(.a(new_n190), .b(new_n199), .c(new_n188), .d(new_n193), .o1(new_n202));
  nor002aa1n02x5               g107(.a(new_n201), .b(new_n202), .o1(\s[16] ));
  nano23aa1n03x5               g108(.a(new_n190), .b(new_n197), .c(new_n198), .d(new_n193), .out0(new_n204));
  nand02aa1n02x5               g109(.a(new_n204), .b(new_n192), .o1(new_n205));
  nor042aa1n06x5               g110(.a(new_n178), .b(new_n205), .o1(new_n206));
  aoai13aa1n12x5               g111(.a(new_n206), .b(new_n132), .c(new_n115), .d(new_n129), .o1(new_n207));
  nano32aa1n06x5               g112(.a(new_n184), .b(new_n199), .c(new_n191), .d(new_n193), .out0(new_n208));
  oai112aa1n03x5               g113(.a(new_n183), .b(new_n193), .c(new_n182), .d(new_n170), .o1(new_n209));
  nona22aa1n03x5               g114(.a(new_n209), .b(new_n197), .c(new_n190), .out0(new_n210));
  aoi022aa1d24x5               g115(.a(new_n208), .b(new_n168), .c(new_n198), .d(new_n210), .o1(new_n211));
  xorc02aa1n12x5               g116(.a(\a[17] ), .b(\b[16] ), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n207), .c(new_n211), .out0(\s[17] ));
  nor002aa1d32x5               g118(.a(\b[16] ), .b(\a[17] ), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(new_n210), .b(new_n198), .o1(new_n216));
  oaib12aa1n03x5               g121(.a(new_n216), .b(new_n205), .c(new_n168), .out0(new_n217));
  aoai13aa1n02x5               g122(.a(new_n212), .b(new_n217), .c(new_n149), .d(new_n206), .o1(new_n218));
  xnrc02aa1n12x5               g123(.a(\b[17] ), .b(\a[18] ), .out0(new_n219));
  xobna2aa1n03x5               g124(.a(new_n219), .b(new_n218), .c(new_n215), .out0(\s[18] ));
  inv000aa1d42x5               g125(.a(\a[17] ), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\a[18] ), .o1(new_n222));
  xroi22aa1d04x5               g127(.a(new_n221), .b(\b[16] ), .c(new_n222), .d(\b[17] ), .out0(new_n223));
  inv000aa1n02x5               g128(.a(new_n223), .o1(new_n224));
  oaoi03aa1n12x5               g129(.a(\a[18] ), .b(\b[17] ), .c(new_n215), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n226), .b(new_n224), .c(new_n207), .d(new_n211), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g133(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g134(.a(\b[18] ), .b(\a[19] ), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nand02aa1n02x5               g136(.a(new_n163), .b(new_n208), .o1(new_n232));
  aoai13aa1n12x5               g137(.a(new_n211), .b(new_n232), .c(new_n176), .d(new_n148), .o1(new_n233));
  nand02aa1n20x5               g138(.a(\b[18] ), .b(\a[19] ), .o1(new_n234));
  nanb02aa1d36x5               g139(.a(new_n230), .b(new_n234), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n236), .b(new_n225), .c(new_n233), .d(new_n223), .o1(new_n237));
  xnrc02aa1n06x5               g142(.a(\b[19] ), .b(\a[20] ), .out0(new_n238));
  aoi012aa1n03x5               g143(.a(new_n238), .b(new_n237), .c(new_n231), .o1(new_n239));
  inv040aa1n02x5               g144(.a(new_n238), .o1(new_n240));
  aoi112aa1n03x4               g145(.a(new_n230), .b(new_n240), .c(new_n227), .d(new_n234), .o1(new_n241));
  nor002aa1n02x5               g146(.a(new_n239), .b(new_n241), .o1(\s[20] ));
  nona23aa1d24x5               g147(.a(new_n240), .b(new_n212), .c(new_n219), .d(new_n235), .out0(new_n243));
  nor042aa1n04x5               g148(.a(\b[17] ), .b(\a[18] ), .o1(new_n244));
  nand42aa1n02x5               g149(.a(\b[17] ), .b(\a[18] ), .o1(new_n245));
  oai112aa1n06x5               g150(.a(new_n245), .b(new_n234), .c(new_n244), .d(new_n214), .o1(new_n246));
  oab012aa1n06x5               g151(.a(new_n230), .b(\a[20] ), .c(\b[19] ), .out0(new_n247));
  aoi022aa1d18x5               g152(.a(new_n246), .b(new_n247), .c(\b[19] ), .d(\a[20] ), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  aoai13aa1n03x5               g154(.a(new_n249), .b(new_n243), .c(new_n207), .d(new_n211), .o1(new_n250));
  xorb03aa1n02x5               g155(.a(new_n250), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g156(.a(\b[20] ), .b(\a[21] ), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n243), .o1(new_n254));
  nand42aa1n02x5               g159(.a(\b[20] ), .b(\a[21] ), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n255), .b(new_n252), .out0(new_n256));
  aoai13aa1n03x5               g161(.a(new_n256), .b(new_n248), .c(new_n233), .d(new_n254), .o1(new_n257));
  inv040aa1d32x5               g162(.a(\a[22] ), .o1(new_n258));
  inv040aa1d28x5               g163(.a(\b[21] ), .o1(new_n259));
  nand22aa1n06x5               g164(.a(new_n259), .b(new_n258), .o1(new_n260));
  nand42aa1n16x5               g165(.a(\b[21] ), .b(\a[22] ), .o1(new_n261));
  nand22aa1n09x5               g166(.a(new_n260), .b(new_n261), .o1(new_n262));
  aoi012aa1n02x7               g167(.a(new_n262), .b(new_n257), .c(new_n253), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n262), .o1(new_n264));
  aoi112aa1n03x4               g169(.a(new_n252), .b(new_n264), .c(new_n250), .d(new_n256), .o1(new_n265));
  nor002aa1n02x5               g170(.a(new_n263), .b(new_n265), .o1(\s[22] ));
  nano22aa1n03x7               g171(.a(new_n262), .b(new_n253), .c(new_n255), .out0(new_n267));
  nano32aa1n02x4               g172(.a(new_n224), .b(new_n267), .c(new_n236), .d(new_n240), .out0(new_n268));
  inv020aa1n02x5               g173(.a(new_n268), .o1(new_n269));
  nanp02aa1n09x5               g174(.a(new_n246), .b(new_n247), .o1(new_n270));
  nanp02aa1n02x5               g175(.a(\b[19] ), .b(\a[20] ), .o1(new_n271));
  nano32aa1n03x7               g176(.a(new_n262), .b(new_n253), .c(new_n255), .d(new_n271), .out0(new_n272));
  oaoi03aa1n09x5               g177(.a(new_n258), .b(new_n259), .c(new_n252), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  aoi012aa1n09x5               g179(.a(new_n274), .b(new_n270), .c(new_n272), .o1(new_n275));
  aoai13aa1n04x5               g180(.a(new_n275), .b(new_n269), .c(new_n207), .d(new_n211), .o1(new_n276));
  xorb03aa1n02x5               g181(.a(new_n276), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g182(.a(\b[22] ), .b(\a[23] ), .o1(new_n278));
  inv000aa1n02x5               g183(.a(new_n278), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n275), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[23] ), .b(\b[22] ), .out0(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n280), .c(new_n233), .d(new_n268), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[24] ), .b(\b[23] ), .out0(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  tech160nm_fiaoi012aa1n02p5x5 g189(.a(new_n284), .b(new_n282), .c(new_n279), .o1(new_n285));
  aoi112aa1n02x7               g190(.a(new_n278), .b(new_n283), .c(new_n276), .d(new_n281), .o1(new_n286));
  norp02aa1n03x5               g191(.a(new_n285), .b(new_n286), .o1(\s[24] ));
  nano32aa1n03x7               g192(.a(new_n243), .b(new_n283), .c(new_n267), .d(new_n281), .out0(new_n288));
  inv000aa1n02x5               g193(.a(new_n288), .o1(new_n289));
  norp02aa1n02x5               g194(.a(new_n244), .b(new_n214), .o1(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n245), .c(new_n234), .out0(new_n291));
  inv020aa1n02x5               g196(.a(new_n247), .o1(new_n292));
  oai012aa1n06x5               g197(.a(new_n272), .b(new_n291), .c(new_n292), .o1(new_n293));
  xnrc02aa1n12x5               g198(.a(\b[22] ), .b(\a[23] ), .out0(new_n294));
  norb02aa1n12x5               g199(.a(new_n283), .b(new_n294), .out0(new_n295));
  inv030aa1n04x5               g200(.a(new_n295), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[24] ), .b(\b[23] ), .c(new_n279), .carry(new_n297));
  aoai13aa1n06x5               g202(.a(new_n297), .b(new_n296), .c(new_n293), .d(new_n273), .o1(new_n298));
  inv040aa1n03x5               g203(.a(new_n298), .o1(new_n299));
  aoai13aa1n04x5               g204(.a(new_n299), .b(new_n289), .c(new_n207), .d(new_n211), .o1(new_n300));
  xorb03aa1n02x5               g205(.a(new_n300), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n06x5               g206(.a(\b[24] ), .b(\a[25] ), .o1(new_n302));
  inv000aa1d42x5               g207(.a(new_n302), .o1(new_n303));
  xorc02aa1n12x5               g208(.a(\a[25] ), .b(\b[24] ), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n298), .c(new_n233), .d(new_n288), .o1(new_n305));
  xorc02aa1n12x5               g210(.a(\a[26] ), .b(\b[25] ), .out0(new_n306));
  inv000aa1d42x5               g211(.a(new_n306), .o1(new_n307));
  aoi012aa1n03x5               g212(.a(new_n307), .b(new_n305), .c(new_n303), .o1(new_n308));
  aoi112aa1n03x4               g213(.a(new_n302), .b(new_n306), .c(new_n300), .d(new_n304), .o1(new_n309));
  nor002aa1n02x5               g214(.a(new_n308), .b(new_n309), .o1(\s[26] ));
  nand02aa1d08x5               g215(.a(new_n306), .b(new_n304), .o1(new_n311));
  nano23aa1d15x5               g216(.a(new_n243), .b(new_n311), .c(new_n295), .d(new_n267), .out0(new_n312));
  aoai13aa1n06x5               g217(.a(new_n312), .b(new_n217), .c(new_n149), .d(new_n206), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n311), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[26] ), .b(\b[25] ), .c(new_n303), .carry(new_n315));
  inv000aa1d42x5               g220(.a(new_n315), .o1(new_n316));
  aoi012aa1n09x5               g221(.a(new_n316), .b(new_n298), .c(new_n314), .o1(new_n317));
  xorc02aa1n12x5               g222(.a(\a[27] ), .b(\b[26] ), .out0(new_n318));
  xnbna2aa1n03x5               g223(.a(new_n318), .b(new_n317), .c(new_n313), .out0(\s[27] ));
  norp02aa1n02x5               g224(.a(\b[26] ), .b(\a[27] ), .o1(new_n320));
  inv040aa1n03x5               g225(.a(new_n320), .o1(new_n321));
  aoai13aa1n03x5               g226(.a(new_n295), .b(new_n274), .c(new_n270), .d(new_n272), .o1(new_n322));
  aoai13aa1n06x5               g227(.a(new_n315), .b(new_n311), .c(new_n322), .d(new_n297), .o1(new_n323));
  aoai13aa1n03x5               g228(.a(new_n318), .b(new_n323), .c(new_n233), .d(new_n312), .o1(new_n324));
  xnrc02aa1n02x5               g229(.a(\b[27] ), .b(\a[28] ), .out0(new_n325));
  tech160nm_fiaoi012aa1n02p5x5 g230(.a(new_n325), .b(new_n324), .c(new_n321), .o1(new_n326));
  aobi12aa1n02x7               g231(.a(new_n318), .b(new_n317), .c(new_n313), .out0(new_n327));
  nano22aa1n03x5               g232(.a(new_n327), .b(new_n321), .c(new_n325), .out0(new_n328));
  norp02aa1n03x5               g233(.a(new_n326), .b(new_n328), .o1(\s[28] ));
  norb02aa1n02x5               g234(.a(new_n318), .b(new_n325), .out0(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n323), .c(new_n233), .d(new_n312), .o1(new_n331));
  oao003aa1n02x5               g236(.a(\a[28] ), .b(\b[27] ), .c(new_n321), .carry(new_n332));
  xnrc02aa1n02x5               g237(.a(\b[28] ), .b(\a[29] ), .out0(new_n333));
  tech160nm_fiaoi012aa1n02p5x5 g238(.a(new_n333), .b(new_n331), .c(new_n332), .o1(new_n334));
  aobi12aa1n02x7               g239(.a(new_n330), .b(new_n317), .c(new_n313), .out0(new_n335));
  nano22aa1n02x4               g240(.a(new_n335), .b(new_n332), .c(new_n333), .out0(new_n336));
  norp02aa1n03x5               g241(.a(new_n334), .b(new_n336), .o1(\s[29] ));
  xorb03aa1n02x5               g242(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g243(.a(new_n318), .b(new_n333), .c(new_n325), .out0(new_n339));
  aoai13aa1n03x5               g244(.a(new_n339), .b(new_n323), .c(new_n233), .d(new_n312), .o1(new_n340));
  oao003aa1n02x5               g245(.a(\a[29] ), .b(\b[28] ), .c(new_n332), .carry(new_n341));
  xnrc02aa1n02x5               g246(.a(\b[29] ), .b(\a[30] ), .out0(new_n342));
  tech160nm_fiaoi012aa1n02p5x5 g247(.a(new_n342), .b(new_n340), .c(new_n341), .o1(new_n343));
  aobi12aa1n02x7               g248(.a(new_n339), .b(new_n317), .c(new_n313), .out0(new_n344));
  nano22aa1n02x4               g249(.a(new_n344), .b(new_n341), .c(new_n342), .out0(new_n345));
  norp02aa1n03x5               g250(.a(new_n343), .b(new_n345), .o1(\s[30] ));
  xnrc02aa1n02x5               g251(.a(\b[30] ), .b(\a[31] ), .out0(new_n347));
  nona32aa1n02x4               g252(.a(new_n318), .b(new_n342), .c(new_n333), .d(new_n325), .out0(new_n348));
  aoi012aa1n02x7               g253(.a(new_n348), .b(new_n317), .c(new_n313), .o1(new_n349));
  oao003aa1n02x5               g254(.a(\a[30] ), .b(\b[29] ), .c(new_n341), .carry(new_n350));
  nano22aa1n02x4               g255(.a(new_n349), .b(new_n347), .c(new_n350), .out0(new_n351));
  inv000aa1n02x5               g256(.a(new_n348), .o1(new_n352));
  aoai13aa1n03x5               g257(.a(new_n352), .b(new_n323), .c(new_n233), .d(new_n312), .o1(new_n353));
  tech160nm_fiaoi012aa1n02p5x5 g258(.a(new_n347), .b(new_n353), .c(new_n350), .o1(new_n354));
  norp02aa1n03x5               g259(.a(new_n354), .b(new_n351), .o1(\s[31] ));
  xnbna2aa1n03x5               g260(.a(new_n102), .b(new_n110), .c(new_n109), .out0(\s[3] ));
  aoai13aa1n02x5               g261(.a(new_n143), .b(new_n99), .c(new_n101), .d(new_n100), .o1(new_n357));
  aoi022aa1n02x5               g262(.a(new_n105), .b(new_n103), .c(new_n107), .d(new_n108), .o1(new_n358));
  aoi022aa1n02x5               g263(.a(new_n115), .b(new_n105), .c(new_n358), .d(new_n357), .o1(\s[4] ));
  xorb03aa1n02x5               g264(.a(new_n115), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoai13aa1n02x5               g265(.a(new_n126), .b(new_n128), .c(new_n144), .d(new_n114), .o1(new_n361));
  xorb03aa1n02x5               g266(.a(new_n361), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  orn002aa1n02x5               g267(.a(new_n123), .b(new_n128), .o(new_n363));
  aoai13aa1n02x5               g268(.a(new_n130), .b(new_n363), .c(new_n144), .d(new_n114), .o1(new_n364));
  xorb03aa1n02x5               g269(.a(new_n364), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g270(.a(new_n119), .b(new_n364), .c(new_n118), .o1(new_n366));
  xnrb03aa1n02x5               g271(.a(new_n366), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g272(.a(new_n149), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


