// Benchmark "adder" written by ABC on Thu Jul 18 01:31:49 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n244, new_n245, new_n246, new_n247, new_n248, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n290, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n348, new_n350, new_n352, new_n354, new_n355,
    new_n358, new_n359;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n06x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nand42aa1n02x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand22aa1n09x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  aob012aa1n06x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .out0(new_n104));
  nor042aa1n06x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nanp02aa1n04x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  norb02aa1n03x5               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  nor042aa1n06x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand42aa1n02x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  norb02aa1n02x5               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  nanp03aa1n02x5               g015(.a(new_n104), .b(new_n107), .c(new_n110), .o1(new_n111));
  tech160nm_fioai012aa1n03p5x5 g016(.a(new_n106), .b(new_n108), .c(new_n105), .o1(new_n112));
  nor042aa1n09x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nand02aa1n04x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nor042aa1n03x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nanp02aa1n04x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nano23aa1n02x4               g021(.a(new_n113), .b(new_n115), .c(new_n116), .d(new_n114), .out0(new_n117));
  nor042aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nanp02aa1n06x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nor042aa1d18x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nand02aa1n04x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  nano23aa1n02x4               g026(.a(new_n118), .b(new_n120), .c(new_n121), .d(new_n119), .out0(new_n122));
  nanp02aa1n02x5               g027(.a(new_n122), .b(new_n117), .o1(new_n123));
  norb02aa1n06x4               g028(.a(new_n119), .b(new_n118), .out0(new_n124));
  norb02aa1n06x4               g029(.a(new_n121), .b(new_n120), .out0(new_n125));
  inv000aa1d42x5               g030(.a(new_n113), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(new_n115), .b(new_n114), .o1(new_n127));
  nand22aa1n03x5               g032(.a(new_n127), .b(new_n126), .o1(new_n128));
  inv000aa1n02x5               g033(.a(new_n120), .o1(new_n129));
  oaoi03aa1n09x5               g034(.a(\a[8] ), .b(\b[7] ), .c(new_n129), .o1(new_n130));
  aoi013aa1n06x4               g035(.a(new_n130), .b(new_n128), .c(new_n125), .d(new_n124), .o1(new_n131));
  aoai13aa1n06x5               g036(.a(new_n131), .b(new_n123), .c(new_n111), .d(new_n112), .o1(new_n132));
  nand42aa1n03x5               g037(.a(\b[8] ), .b(\a[9] ), .o1(new_n133));
  aoi012aa1n02x5               g038(.a(new_n98), .b(new_n132), .c(new_n133), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  nor002aa1n03x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  nanp02aa1n06x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  nano23aa1n06x5               g042(.a(new_n136), .b(new_n98), .c(new_n133), .d(new_n137), .out0(new_n138));
  nanp02aa1n02x5               g043(.a(new_n98), .b(new_n137), .o1(new_n139));
  oaib12aa1n02x5               g044(.a(new_n139), .b(\b[9] ), .c(new_n97), .out0(new_n140));
  nor002aa1d32x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand42aa1n03x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n06x4               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n140), .c(new_n132), .d(new_n138), .o1(new_n144));
  aoi112aa1n02x5               g049(.a(new_n143), .b(new_n140), .c(new_n132), .d(new_n138), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n144), .b(new_n145), .out0(\s[11] ));
  inv000aa1d42x5               g051(.a(new_n141), .o1(new_n147));
  nor002aa1n02x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nanp02aa1n04x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  norb02aa1n03x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  xnbna2aa1n03x5               g055(.a(new_n150), .b(new_n144), .c(new_n147), .out0(\s[12] ));
  oaoi03aa1n02x5               g056(.a(new_n99), .b(new_n100), .c(new_n103), .o1(new_n152));
  nona23aa1n03x5               g057(.a(new_n109), .b(new_n106), .c(new_n105), .d(new_n108), .out0(new_n153));
  oai012aa1n04x7               g058(.a(new_n112), .b(new_n153), .c(new_n152), .o1(new_n154));
  nona23aa1n02x4               g059(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n155));
  nano22aa1n03x7               g060(.a(new_n155), .b(new_n124), .c(new_n125), .out0(new_n156));
  nanp03aa1n02x5               g061(.a(new_n128), .b(new_n124), .c(new_n125), .o1(new_n157));
  nanb02aa1n06x5               g062(.a(new_n130), .b(new_n157), .out0(new_n158));
  and003aa1n02x5               g063(.a(new_n138), .b(new_n150), .c(new_n143), .o(new_n159));
  aoai13aa1n06x5               g064(.a(new_n159), .b(new_n158), .c(new_n154), .d(new_n156), .o1(new_n160));
  nona22aa1n03x5               g065(.a(new_n139), .b(new_n141), .c(new_n136), .out0(new_n161));
  aoai13aa1n06x5               g066(.a(new_n149), .b(new_n148), .c(new_n161), .d(new_n142), .o1(new_n162));
  nor042aa1n06x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nanp02aa1n04x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n160), .c(new_n162), .out0(\s[13] ));
  nanp02aa1n02x5               g071(.a(new_n160), .b(new_n162), .o1(new_n167));
  aoi012aa1n02x5               g072(.a(new_n163), .b(new_n167), .c(new_n164), .o1(new_n168));
  xnrb03aa1n03x5               g073(.a(new_n168), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n06x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand02aa1n06x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nano23aa1n06x5               g076(.a(new_n163), .b(new_n170), .c(new_n171), .d(new_n164), .out0(new_n172));
  nanp02aa1n02x5               g077(.a(new_n167), .b(new_n172), .o1(new_n173));
  aoi012aa1n02x5               g078(.a(new_n170), .b(new_n163), .c(new_n171), .o1(new_n174));
  nor042aa1n03x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nand02aa1n16x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  norb02aa1n03x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  xnbna2aa1n03x5               g082(.a(new_n177), .b(new_n173), .c(new_n174), .out0(\s[15] ));
  nona23aa1n03x5               g083(.a(new_n171), .b(new_n164), .c(new_n163), .d(new_n170), .out0(new_n179));
  aoai13aa1n04x5               g084(.a(new_n174), .b(new_n179), .c(new_n160), .d(new_n162), .o1(new_n180));
  nor042aa1n09x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  and002aa1n24x5               g086(.a(\b[15] ), .b(\a[16] ), .o(new_n182));
  nor042aa1d18x5               g087(.a(new_n182), .b(new_n181), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n184), .b(new_n175), .c(new_n180), .d(new_n176), .o1(new_n185));
  aoi112aa1n03x5               g090(.a(new_n175), .b(new_n184), .c(new_n180), .d(new_n176), .o1(new_n186));
  nanb02aa1n02x5               g091(.a(new_n186), .b(new_n185), .out0(\s[16] ));
  nand03aa1n04x5               g092(.a(new_n172), .b(new_n177), .c(new_n183), .o1(new_n188));
  nano32aa1n03x7               g093(.a(new_n188), .b(new_n150), .c(new_n138), .d(new_n143), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n158), .c(new_n154), .d(new_n156), .o1(new_n190));
  aoi112aa1n02x5               g095(.a(new_n141), .b(new_n136), .c(new_n98), .d(new_n137), .o1(new_n191));
  obai22aa1n03x5               g096(.a(new_n142), .b(new_n191), .c(\a[12] ), .d(\b[11] ), .out0(new_n192));
  nano22aa1n03x7               g097(.a(new_n179), .b(new_n183), .c(new_n177), .out0(new_n193));
  inv000aa1d42x5               g098(.a(new_n176), .o1(new_n194));
  inv000aa1d42x5               g099(.a(new_n181), .o1(new_n195));
  aoi112aa1n03x4               g100(.a(new_n170), .b(new_n175), .c(new_n163), .d(new_n171), .o1(new_n196));
  oaoi13aa1n02x5               g101(.a(new_n182), .b(new_n195), .c(new_n196), .d(new_n194), .o1(new_n197));
  aoi013aa1n06x4               g102(.a(new_n197), .b(new_n192), .c(new_n193), .d(new_n149), .o1(new_n198));
  nand02aa1d08x5               g103(.a(new_n190), .b(new_n198), .o1(new_n199));
  xorc02aa1n02x5               g104(.a(\a[17] ), .b(\b[16] ), .out0(new_n200));
  aoi113aa1n02x5               g105(.a(new_n200), .b(new_n197), .c(new_n192), .d(new_n193), .e(new_n149), .o1(new_n201));
  aoi022aa1n02x5               g106(.a(new_n199), .b(new_n200), .c(new_n190), .d(new_n201), .o1(\s[17] ));
  inv040aa1d32x5               g107(.a(\a[18] ), .o1(new_n203));
  norp02aa1n12x5               g108(.a(\b[16] ), .b(\a[17] ), .o1(new_n204));
  tech160nm_fiaoi012aa1n05x5   g109(.a(new_n204), .b(new_n199), .c(new_n200), .o1(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[17] ), .c(new_n203), .out0(\s[18] ));
  inv000aa1d42x5               g111(.a(new_n182), .o1(new_n207));
  oai012aa1n02x5               g112(.a(new_n195), .b(new_n196), .c(new_n194), .o1(new_n208));
  nand42aa1n03x5               g113(.a(new_n208), .b(new_n207), .o1(new_n209));
  tech160nm_fioai012aa1n04x5   g114(.a(new_n209), .b(new_n162), .c(new_n188), .o1(new_n210));
  inv000aa1d42x5               g115(.a(\a[17] ), .o1(new_n211));
  xroi22aa1d06x4               g116(.a(new_n211), .b(\b[16] ), .c(new_n203), .d(\b[17] ), .out0(new_n212));
  aoai13aa1n03x5               g117(.a(new_n212), .b(new_n210), .c(new_n132), .d(new_n189), .o1(new_n213));
  nanp02aa1n04x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  nor022aa1n08x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  nor022aa1n04x5               g120(.a(new_n215), .b(new_n204), .o1(new_n216));
  norb02aa1n06x4               g121(.a(new_n214), .b(new_n216), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  norp02aa1n09x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  nand02aa1d04x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  norb02aa1n06x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  xnbna2aa1n03x5               g126(.a(new_n221), .b(new_n213), .c(new_n218), .out0(\s[19] ));
  xnrc02aa1n02x5               g127(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n03x5               g128(.a(new_n213), .b(new_n218), .o1(new_n224));
  nor042aa1n12x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nand02aa1d28x5               g130(.a(\b[19] ), .b(\a[20] ), .o1(new_n226));
  nanb02aa1n06x5               g131(.a(new_n225), .b(new_n226), .out0(new_n227));
  aoai13aa1n02x5               g132(.a(new_n227), .b(new_n219), .c(new_n224), .d(new_n220), .o1(new_n228));
  aoai13aa1n03x5               g133(.a(new_n221), .b(new_n217), .c(new_n199), .d(new_n212), .o1(new_n229));
  nona22aa1n03x5               g134(.a(new_n229), .b(new_n227), .c(new_n219), .out0(new_n230));
  nanp02aa1n03x5               g135(.a(new_n228), .b(new_n230), .o1(\s[20] ));
  nanb03aa1d18x5               g136(.a(new_n227), .b(new_n212), .c(new_n221), .out0(new_n232));
  nanb03aa1n06x5               g137(.a(new_n225), .b(new_n226), .c(new_n220), .out0(new_n233));
  orn002aa1n02x5               g138(.a(\a[19] ), .b(\b[18] ), .o(new_n234));
  oai112aa1n04x5               g139(.a(new_n234), .b(new_n214), .c(new_n215), .d(new_n204), .o1(new_n235));
  aoi012aa1d18x5               g140(.a(new_n225), .b(new_n219), .c(new_n226), .o1(new_n236));
  oai012aa1n18x5               g141(.a(new_n236), .b(new_n235), .c(new_n233), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n232), .c(new_n190), .d(new_n198), .o1(new_n239));
  tech160nm_fixorc02aa1n05x5   g144(.a(\a[21] ), .b(\b[20] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n232), .o1(new_n241));
  aoi112aa1n02x5               g146(.a(new_n240), .b(new_n237), .c(new_n199), .d(new_n241), .o1(new_n242));
  aoi012aa1n02x5               g147(.a(new_n242), .b(new_n239), .c(new_n240), .o1(\s[21] ));
  nor042aa1d18x5               g148(.a(\b[20] ), .b(\a[21] ), .o1(new_n244));
  xnrc02aa1n12x5               g149(.a(\b[21] ), .b(\a[22] ), .out0(new_n245));
  aoai13aa1n02x5               g150(.a(new_n245), .b(new_n244), .c(new_n239), .d(new_n240), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(new_n239), .b(new_n240), .o1(new_n247));
  nona22aa1n02x4               g152(.a(new_n247), .b(new_n245), .c(new_n244), .out0(new_n248));
  nanp02aa1n02x5               g153(.a(new_n248), .b(new_n246), .o1(\s[22] ));
  xnrc02aa1n02x5               g154(.a(\b[20] ), .b(\a[21] ), .out0(new_n250));
  nona32aa1n02x4               g155(.a(new_n199), .b(new_n245), .c(new_n250), .d(new_n232), .out0(new_n251));
  nor022aa1n06x5               g156(.a(new_n245), .b(new_n250), .o1(new_n252));
  nanb02aa1n03x5               g157(.a(new_n232), .b(new_n252), .out0(new_n253));
  nano22aa1n03x7               g158(.a(new_n225), .b(new_n220), .c(new_n226), .out0(new_n254));
  oai012aa1n04x7               g159(.a(new_n214), .b(\b[18] ), .c(\a[19] ), .o1(new_n255));
  nona22aa1n03x5               g160(.a(new_n254), .b(new_n216), .c(new_n255), .out0(new_n256));
  nanb02aa1n02x5               g161(.a(new_n245), .b(new_n240), .out0(new_n257));
  inv000aa1d42x5               g162(.a(\a[22] ), .o1(new_n258));
  inv040aa1n08x5               g163(.a(\b[21] ), .o1(new_n259));
  oao003aa1n03x5               g164(.a(new_n258), .b(new_n259), .c(new_n244), .carry(new_n260));
  inv040aa1n02x5               g165(.a(new_n260), .o1(new_n261));
  aoai13aa1n12x5               g166(.a(new_n261), .b(new_n257), .c(new_n256), .d(new_n236), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  aoai13aa1n04x5               g168(.a(new_n263), .b(new_n253), .c(new_n190), .d(new_n198), .o1(new_n264));
  xorc02aa1n12x5               g169(.a(\a[23] ), .b(\b[22] ), .out0(new_n265));
  aoi112aa1n02x5               g170(.a(new_n265), .b(new_n260), .c(new_n237), .d(new_n252), .o1(new_n266));
  aoi022aa1n02x5               g171(.a(new_n266), .b(new_n251), .c(new_n264), .d(new_n265), .o1(\s[23] ));
  nor002aa1n02x5               g172(.a(\b[22] ), .b(\a[23] ), .o1(new_n268));
  xnrc02aa1n12x5               g173(.a(\b[23] ), .b(\a[24] ), .out0(new_n269));
  aoai13aa1n03x5               g174(.a(new_n269), .b(new_n268), .c(new_n264), .d(new_n265), .o1(new_n270));
  nanp02aa1n02x5               g175(.a(new_n264), .b(new_n265), .o1(new_n271));
  nona22aa1n02x5               g176(.a(new_n271), .b(new_n269), .c(new_n268), .out0(new_n272));
  nanp02aa1n03x5               g177(.a(new_n272), .b(new_n270), .o1(\s[24] ));
  norb02aa1n15x5               g178(.a(new_n265), .b(new_n269), .out0(new_n274));
  nano22aa1n03x7               g179(.a(new_n232), .b(new_n252), .c(new_n274), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n210), .c(new_n132), .d(new_n189), .o1(new_n276));
  oab012aa1n02x4               g181(.a(new_n255), .b(new_n204), .c(new_n215), .out0(new_n277));
  inv000aa1d42x5               g182(.a(new_n236), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n252), .b(new_n278), .c(new_n277), .d(new_n254), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n274), .o1(new_n280));
  inv000aa1d42x5               g185(.a(\a[24] ), .o1(new_n281));
  inv000aa1d42x5               g186(.a(\b[23] ), .o1(new_n282));
  oao003aa1n02x5               g187(.a(new_n281), .b(new_n282), .c(new_n268), .carry(new_n283));
  inv030aa1n02x5               g188(.a(new_n283), .o1(new_n284));
  aoai13aa1n06x5               g189(.a(new_n284), .b(new_n280), .c(new_n279), .d(new_n261), .o1(new_n285));
  nanb02aa1n06x5               g190(.a(new_n285), .b(new_n276), .out0(new_n286));
  xorc02aa1n12x5               g191(.a(\a[25] ), .b(\b[24] ), .out0(new_n287));
  aoi112aa1n02x5               g192(.a(new_n287), .b(new_n283), .c(new_n262), .d(new_n274), .o1(new_n288));
  aoi022aa1n02x5               g193(.a(new_n286), .b(new_n287), .c(new_n276), .d(new_n288), .o1(\s[25] ));
  norp02aa1n02x5               g194(.a(\b[24] ), .b(\a[25] ), .o1(new_n290));
  norp02aa1n03x5               g195(.a(\b[25] ), .b(\a[26] ), .o1(new_n291));
  nand42aa1n03x5               g196(.a(\b[25] ), .b(\a[26] ), .o1(new_n292));
  nanb02aa1n06x5               g197(.a(new_n291), .b(new_n292), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n290), .c(new_n286), .d(new_n287), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n287), .b(new_n285), .c(new_n199), .d(new_n275), .o1(new_n295));
  nona22aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n290), .out0(new_n296));
  nanp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(\s[26] ));
  norb02aa1n09x5               g202(.a(new_n287), .b(new_n293), .out0(new_n298));
  nano32aa1d12x5               g203(.a(new_n232), .b(new_n298), .c(new_n252), .d(new_n274), .out0(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n210), .c(new_n132), .d(new_n189), .o1(new_n300));
  aoai13aa1n04x5               g205(.a(new_n298), .b(new_n283), .c(new_n262), .d(new_n274), .o1(new_n301));
  oai012aa1n02x5               g206(.a(new_n292), .b(new_n291), .c(new_n290), .o1(new_n302));
  nand43aa1n03x5               g207(.a(new_n300), .b(new_n301), .c(new_n302), .o1(new_n303));
  xorc02aa1n12x5               g208(.a(\a[27] ), .b(\b[26] ), .out0(new_n304));
  nano22aa1n02x4               g209(.a(new_n304), .b(new_n301), .c(new_n302), .out0(new_n305));
  aoi022aa1n02x5               g210(.a(new_n305), .b(new_n300), .c(new_n303), .d(new_n304), .o1(\s[27] ));
  norp02aa1n02x5               g211(.a(\b[26] ), .b(\a[27] ), .o1(new_n307));
  norp02aa1n04x5               g212(.a(\b[27] ), .b(\a[28] ), .o1(new_n308));
  nand22aa1n03x5               g213(.a(\b[27] ), .b(\a[28] ), .o1(new_n309));
  nanb02aa1n06x5               g214(.a(new_n308), .b(new_n309), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n307), .c(new_n303), .d(new_n304), .o1(new_n311));
  aoai13aa1n06x5               g216(.a(new_n274), .b(new_n260), .c(new_n237), .d(new_n252), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n298), .o1(new_n313));
  aoai13aa1n06x5               g218(.a(new_n302), .b(new_n313), .c(new_n312), .d(new_n284), .o1(new_n314));
  aoai13aa1n03x5               g219(.a(new_n304), .b(new_n314), .c(new_n199), .d(new_n299), .o1(new_n315));
  nona22aa1n02x5               g220(.a(new_n315), .b(new_n310), .c(new_n307), .out0(new_n316));
  nanp02aa1n03x5               g221(.a(new_n311), .b(new_n316), .o1(\s[28] ));
  norb02aa1d27x5               g222(.a(new_n304), .b(new_n310), .out0(new_n318));
  aoai13aa1n02x5               g223(.a(new_n318), .b(new_n314), .c(new_n199), .d(new_n299), .o1(new_n319));
  xorc02aa1n12x5               g224(.a(\a[29] ), .b(\b[28] ), .out0(new_n320));
  aoi012aa1n02x5               g225(.a(new_n308), .b(new_n307), .c(new_n309), .o1(new_n321));
  norb02aa1n02x5               g226(.a(new_n321), .b(new_n320), .out0(new_n322));
  aobi12aa1n02x5               g227(.a(new_n302), .b(new_n285), .c(new_n298), .out0(new_n323));
  inv000aa1d42x5               g228(.a(new_n318), .o1(new_n324));
  aoai13aa1n02x7               g229(.a(new_n321), .b(new_n324), .c(new_n323), .d(new_n300), .o1(new_n325));
  aoi022aa1n03x5               g230(.a(new_n325), .b(new_n320), .c(new_n319), .d(new_n322), .o1(\s[29] ));
  xorb03aa1n02x5               g231(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g232(.a(new_n310), .b(new_n304), .c(new_n320), .out0(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n314), .c(new_n199), .d(new_n299), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .out0(new_n330));
  oao003aa1n09x5               g235(.a(\a[29] ), .b(\b[28] ), .c(new_n321), .carry(new_n331));
  norb02aa1n02x5               g236(.a(new_n331), .b(new_n330), .out0(new_n332));
  inv000aa1n02x5               g237(.a(new_n328), .o1(new_n333));
  aoai13aa1n02x7               g238(.a(new_n331), .b(new_n333), .c(new_n323), .d(new_n300), .o1(new_n334));
  aoi022aa1n03x5               g239(.a(new_n334), .b(new_n330), .c(new_n329), .d(new_n332), .o1(\s[30] ));
  nand03aa1n02x5               g240(.a(new_n318), .b(new_n320), .c(new_n330), .o1(new_n336));
  nanb02aa1n03x5               g241(.a(new_n336), .b(new_n303), .out0(new_n337));
  xorc02aa1n02x5               g242(.a(\a[31] ), .b(\b[30] ), .out0(new_n338));
  inv000aa1d42x5               g243(.a(\a[30] ), .o1(new_n339));
  inv000aa1d42x5               g244(.a(\b[29] ), .o1(new_n340));
  inv000aa1d42x5               g245(.a(new_n331), .o1(new_n341));
  oabi12aa1n02x5               g246(.a(new_n338), .b(\a[30] ), .c(\b[29] ), .out0(new_n342));
  oaoi13aa1n02x5               g247(.a(new_n342), .b(new_n341), .c(new_n339), .d(new_n340), .o1(new_n343));
  oaoi03aa1n02x5               g248(.a(new_n339), .b(new_n340), .c(new_n341), .o1(new_n344));
  aoai13aa1n02x7               g249(.a(new_n344), .b(new_n336), .c(new_n323), .d(new_n300), .o1(new_n345));
  aoi022aa1n03x5               g250(.a(new_n345), .b(new_n338), .c(new_n337), .d(new_n343), .o1(\s[31] ));
  xorb03aa1n02x5               g251(.a(new_n104), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi112aa1n02x5               g252(.a(new_n108), .b(new_n107), .c(new_n104), .d(new_n109), .o1(new_n348));
  aoib12aa1n02x5               g253(.a(new_n348), .b(new_n154), .c(new_n105), .out0(\s[4] ));
  nanb02aa1n02x5               g254(.a(new_n115), .b(new_n116), .out0(new_n350));
  xobna2aa1n03x5               g255(.a(new_n350), .b(new_n111), .c(new_n112), .out0(\s[5] ));
  tech160nm_fiao0012aa1n02p5x5 g256(.a(new_n115), .b(new_n154), .c(new_n116), .o(new_n352));
  xorb03aa1n02x5               g257(.a(new_n352), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n03x5               g258(.a(new_n125), .b(new_n113), .c(new_n352), .d(new_n114), .o1(new_n354));
  aoi112aa1n02x5               g259(.a(new_n125), .b(new_n113), .c(new_n352), .d(new_n114), .o1(new_n355));
  norb02aa1n02x5               g260(.a(new_n354), .b(new_n355), .out0(\s[7] ));
  xnbna2aa1n03x5               g261(.a(new_n124), .b(new_n354), .c(new_n129), .out0(\s[8] ));
  norb02aa1n02x5               g262(.a(new_n133), .b(new_n98), .out0(new_n358));
  aoi112aa1n02x5               g263(.a(new_n158), .b(new_n358), .c(new_n154), .d(new_n156), .o1(new_n359));
  aoi012aa1n02x5               g264(.a(new_n359), .b(new_n132), .c(new_n358), .o1(\s[9] ));
endmodule


