// Benchmark "adder" written by ABC on Wed Jul 17 18:57:07 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n290, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n348, new_n349, new_n352, new_n354, new_n356;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(\a[9] ), .o1(new_n98));
  nanb02aa1n02x5               g003(.a(\b[8] ), .b(new_n98), .out0(new_n99));
  norp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  tech160nm_fiaoi012aa1n04x5   g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .out0(new_n104));
  xnrc02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .out0(new_n105));
  inv000aa1d42x5               g010(.a(\a[4] ), .o1(new_n106));
  inv000aa1d42x5               g011(.a(\b[3] ), .o1(new_n107));
  nor002aa1n04x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  oaoi03aa1n12x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  oai013aa1n03x5               g014(.a(new_n109), .b(new_n104), .c(new_n105), .d(new_n103), .o1(new_n110));
  orn002aa1n02x5               g015(.a(\a[8] ), .b(\b[7] ), .o(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nona23aa1n03x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  nor042aa1n04x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nand42aa1n03x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  norb02aa1n06x5               g024(.a(new_n119), .b(new_n118), .out0(new_n120));
  nano32aa1n02x5               g025(.a(new_n117), .b(new_n120), .c(new_n111), .d(new_n112), .out0(new_n121));
  nanp02aa1n02x5               g026(.a(new_n118), .b(new_n112), .o1(new_n122));
  oai022aa1n02x5               g027(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n123));
  nanb03aa1n02x5               g028(.a(new_n118), .b(new_n119), .c(new_n114), .out0(new_n124));
  nano32aa1n03x7               g029(.a(new_n124), .b(new_n123), .c(new_n111), .d(new_n112), .out0(new_n125));
  nano22aa1n06x5               g030(.a(new_n125), .b(new_n111), .c(new_n122), .out0(new_n126));
  inv030aa1n02x5               g031(.a(new_n126), .o1(new_n127));
  tech160nm_fixorc02aa1n02p5x5 g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n127), .c(new_n110), .d(new_n121), .o1(new_n129));
  xobna2aa1n03x5               g034(.a(new_n97), .b(new_n129), .c(new_n99), .out0(\s[10] ));
  nand42aa1n04x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  inv040aa1n02x5               g036(.a(new_n131), .o1(new_n132));
  nor042aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nanb02aa1n06x5               g039(.a(new_n133), .b(new_n134), .out0(new_n135));
  oai022aa1d18x5               g040(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  aoi112aa1n03x5               g042(.a(new_n135), .b(new_n132), .c(new_n129), .d(new_n137), .o1(new_n138));
  aoai13aa1n02x5               g043(.a(new_n135), .b(new_n132), .c(new_n129), .d(new_n137), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(\s[11] ));
  nor022aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  and002aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o(new_n142));
  norp02aa1n03x5               g047(.a(new_n142), .b(new_n141), .o1(new_n143));
  oai012aa1n02x5               g048(.a(new_n143), .b(new_n138), .c(new_n133), .o1(new_n144));
  norp03aa1n02x5               g049(.a(new_n138), .b(new_n143), .c(new_n133), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n144), .b(new_n145), .out0(\s[12] ));
  norp03aa1n02x5               g051(.a(new_n105), .b(new_n104), .c(new_n103), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n109), .o1(new_n148));
  oai012aa1n02x5               g053(.a(new_n121), .b(new_n147), .c(new_n148), .o1(new_n149));
  nona23aa1n02x4               g054(.a(new_n143), .b(new_n128), .c(new_n97), .d(new_n135), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n141), .o1(new_n151));
  aob012aa1n02x5               g056(.a(new_n133), .b(\b[11] ), .c(\a[12] ), .out0(new_n152));
  nano23aa1n02x4               g057(.a(new_n135), .b(new_n142), .c(new_n136), .d(new_n131), .out0(new_n153));
  nano22aa1n02x4               g058(.a(new_n153), .b(new_n151), .c(new_n152), .out0(new_n154));
  aoai13aa1n02x5               g059(.a(new_n154), .b(new_n150), .c(new_n149), .d(new_n126), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n16x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n157), .b(new_n155), .c(new_n158), .o1(new_n159));
  xnrb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norb02aa1n02x5               g065(.a(new_n134), .b(new_n133), .out0(new_n161));
  nano32aa1n02x4               g066(.a(new_n97), .b(new_n143), .c(new_n128), .d(new_n161), .out0(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n127), .c(new_n110), .d(new_n121), .o1(new_n163));
  nor022aa1n06x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand42aa1n02x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nona23aa1n03x5               g070(.a(new_n165), .b(new_n158), .c(new_n157), .d(new_n164), .out0(new_n166));
  oai012aa1n12x5               g071(.a(new_n165), .b(new_n164), .c(new_n157), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n166), .c(new_n163), .d(new_n154), .o1(new_n168));
  xorb03aa1n02x5               g073(.a(new_n168), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  inv000aa1d42x5               g074(.a(\a[15] ), .o1(new_n170));
  inv000aa1d42x5               g075(.a(\b[14] ), .o1(new_n171));
  nand42aa1n16x5               g076(.a(new_n171), .b(new_n170), .o1(new_n172));
  nano23aa1n02x4               g077(.a(new_n157), .b(new_n164), .c(new_n165), .d(new_n158), .out0(new_n173));
  inv000aa1d42x5               g078(.a(new_n167), .o1(new_n174));
  nand42aa1n03x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(new_n172), .b(new_n175), .o1(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  aoai13aa1n02x5               g082(.a(new_n177), .b(new_n174), .c(new_n155), .d(new_n173), .o1(new_n178));
  xorc02aa1n02x5               g083(.a(\a[16] ), .b(\b[15] ), .out0(new_n179));
  aobi12aa1n02x5               g084(.a(new_n179), .b(new_n178), .c(new_n172), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n172), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(new_n181), .b(new_n179), .c(new_n168), .d(new_n177), .o1(new_n182));
  norp02aa1n02x5               g087(.a(new_n180), .b(new_n182), .o1(\s[16] ));
  norp02aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\a[16] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\b[15] ), .o1(new_n186));
  oai112aa1n06x5               g091(.a(new_n172), .b(new_n175), .c(new_n186), .d(new_n185), .o1(new_n187));
  nor003aa1n02x5               g092(.a(new_n166), .b(new_n184), .c(new_n187), .o1(new_n188));
  nand22aa1n02x5               g093(.a(new_n162), .b(new_n188), .o1(new_n189));
  nona23aa1n02x4               g094(.a(new_n161), .b(new_n136), .c(new_n142), .d(new_n132), .out0(new_n190));
  nanp03aa1n02x5               g095(.a(new_n190), .b(new_n151), .c(new_n152), .o1(new_n191));
  oai112aa1n02x5               g096(.a(new_n170), .b(new_n171), .c(new_n186), .d(new_n185), .o1(new_n192));
  oai122aa1n09x5               g097(.a(new_n192), .b(new_n187), .c(new_n167), .d(\b[15] ), .e(\a[16] ), .o1(new_n193));
  tech160nm_fiaoi012aa1n05x5   g098(.a(new_n193), .b(new_n191), .c(new_n188), .o1(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n189), .c(new_n149), .d(new_n126), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d28x5               g101(.a(\a[17] ), .o1(new_n197));
  inv040aa1d32x5               g102(.a(\b[16] ), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(new_n198), .b(new_n197), .o1(new_n199));
  xorc02aa1n02x5               g104(.a(\a[4] ), .b(\b[3] ), .out0(new_n200));
  nona22aa1n02x4               g105(.a(new_n200), .b(new_n105), .c(new_n103), .out0(new_n201));
  xorc02aa1n02x5               g106(.a(\a[8] ), .b(\b[7] ), .out0(new_n202));
  nanb03aa1n02x5               g107(.a(new_n117), .b(new_n120), .c(new_n202), .out0(new_n203));
  aoai13aa1n02x5               g108(.a(new_n126), .b(new_n203), .c(new_n201), .d(new_n109), .o1(new_n204));
  nona22aa1n02x4               g109(.a(new_n173), .b(new_n187), .c(new_n184), .out0(new_n205));
  nor042aa1n03x5               g110(.a(new_n150), .b(new_n205), .o1(new_n206));
  oabi12aa1n02x5               g111(.a(new_n193), .b(new_n154), .c(new_n205), .out0(new_n207));
  tech160nm_fixorc02aa1n04x5   g112(.a(\a[17] ), .b(\b[16] ), .out0(new_n208));
  aoai13aa1n02x5               g113(.a(new_n208), .b(new_n207), .c(new_n204), .d(new_n206), .o1(new_n209));
  nor022aa1n16x5               g114(.a(\b[17] ), .b(\a[18] ), .o1(new_n210));
  nand42aa1d28x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  nanb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(new_n212));
  xobna2aa1n03x5               g117(.a(new_n212), .b(new_n209), .c(new_n199), .out0(\s[18] ));
  aoai13aa1n06x5               g118(.a(new_n206), .b(new_n127), .c(new_n110), .d(new_n121), .o1(new_n214));
  norb02aa1n02x7               g119(.a(new_n208), .b(new_n212), .out0(new_n215));
  inv000aa1n02x5               g120(.a(new_n215), .o1(new_n216));
  aoai13aa1n12x5               g121(.a(new_n211), .b(new_n210), .c(new_n197), .d(new_n198), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n216), .c(new_n214), .d(new_n194), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1d18x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n217), .o1(new_n223));
  nanp02aa1n04x5               g128(.a(\b[18] ), .b(\a[19] ), .o1(new_n224));
  nanb02aa1n02x5               g129(.a(new_n221), .b(new_n224), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n226), .b(new_n223), .c(new_n195), .d(new_n215), .o1(new_n227));
  xorc02aa1n02x5               g132(.a(\a[20] ), .b(\b[19] ), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  tech160nm_fiaoi012aa1n02p5x5 g134(.a(new_n229), .b(new_n227), .c(new_n222), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n221), .b(new_n228), .c(new_n218), .d(new_n226), .o1(new_n231));
  nor002aa1n02x5               g136(.a(new_n230), .b(new_n231), .o1(\s[20] ));
  inv040aa1d30x5               g137(.a(\a[20] ), .o1(new_n233));
  inv000aa1d42x5               g138(.a(\b[19] ), .o1(new_n234));
  nand42aa1n04x5               g139(.a(new_n234), .b(new_n233), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  nanp02aa1n04x5               g141(.a(\b[19] ), .b(\a[20] ), .o1(new_n237));
  nanb03aa1d18x5               g142(.a(new_n221), .b(new_n237), .c(new_n224), .out0(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  nona23aa1d18x5               g144(.a(new_n239), .b(new_n208), .c(new_n212), .d(new_n236), .out0(new_n240));
  nanp02aa1n02x5               g145(.a(new_n221), .b(new_n237), .o1(new_n241));
  oai112aa1n06x5               g146(.a(new_n241), .b(new_n235), .c(new_n238), .d(new_n217), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoai13aa1n02x5               g148(.a(new_n243), .b(new_n240), .c(new_n214), .d(new_n194), .o1(new_n244));
  xorb03aa1n02x5               g149(.a(new_n244), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n16x5               g150(.a(\b[20] ), .b(\a[21] ), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n240), .o1(new_n248));
  nanp02aa1n02x5               g153(.a(\b[20] ), .b(\a[21] ), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n246), .out0(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n242), .c(new_n195), .d(new_n248), .o1(new_n251));
  nor022aa1n04x5               g156(.a(\b[21] ), .b(\a[22] ), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(\b[21] ), .b(\a[22] ), .o1(new_n253));
  norb02aa1n02x5               g158(.a(new_n253), .b(new_n252), .out0(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  aoi012aa1n03x5               g160(.a(new_n255), .b(new_n251), .c(new_n247), .o1(new_n256));
  aoi112aa1n02x5               g161(.a(new_n246), .b(new_n254), .c(new_n244), .d(new_n250), .o1(new_n257));
  norp02aa1n02x5               g162(.a(new_n256), .b(new_n257), .o1(\s[22] ));
  nona23aa1d18x5               g163(.a(new_n253), .b(new_n249), .c(new_n246), .d(new_n252), .out0(new_n259));
  norp02aa1n02x5               g164(.a(new_n240), .b(new_n259), .o1(new_n260));
  inv000aa1n02x5               g165(.a(new_n260), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n259), .o1(new_n262));
  oai012aa1n02x5               g167(.a(new_n253), .b(new_n252), .c(new_n246), .o1(new_n263));
  aobi12aa1n18x5               g168(.a(new_n263), .b(new_n242), .c(new_n262), .out0(new_n264));
  aoai13aa1n04x5               g169(.a(new_n264), .b(new_n261), .c(new_n214), .d(new_n194), .o1(new_n265));
  xorb03aa1n02x5               g170(.a(new_n265), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  orn002aa1n06x5               g171(.a(\a[23] ), .b(\b[22] ), .o(new_n267));
  inv000aa1d42x5               g172(.a(new_n264), .o1(new_n268));
  nanp02aa1n02x5               g173(.a(\b[22] ), .b(\a[23] ), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(new_n267), .b(new_n269), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aoai13aa1n02x5               g176(.a(new_n271), .b(new_n268), .c(new_n195), .d(new_n260), .o1(new_n272));
  norp02aa1n02x5               g177(.a(\b[23] ), .b(\a[24] ), .o1(new_n273));
  nanp02aa1n02x5               g178(.a(\b[23] ), .b(\a[24] ), .o1(new_n274));
  norb02aa1n02x5               g179(.a(new_n274), .b(new_n273), .out0(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  aoi012aa1n02x5               g181(.a(new_n276), .b(new_n272), .c(new_n267), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n267), .o1(new_n278));
  aoi112aa1n02x5               g183(.a(new_n278), .b(new_n275), .c(new_n265), .d(new_n271), .o1(new_n279));
  nor002aa1n02x5               g184(.a(new_n277), .b(new_n279), .o1(\s[24] ));
  nanp03aa1n02x5               g185(.a(new_n267), .b(new_n269), .c(new_n274), .o1(new_n281));
  nor043aa1n03x5               g186(.a(new_n259), .b(new_n273), .c(new_n281), .o1(new_n282));
  norb02aa1n03x5               g187(.a(new_n282), .b(new_n240), .out0(new_n283));
  inv020aa1n02x5               g188(.a(new_n283), .o1(new_n284));
  aoi112aa1n02x5               g189(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n285));
  oai022aa1n02x5               g190(.a(new_n281), .b(new_n263), .c(\b[23] ), .d(\a[24] ), .o1(new_n286));
  aoi112aa1n06x5               g191(.a(new_n286), .b(new_n285), .c(new_n242), .d(new_n282), .o1(new_n287));
  aoai13aa1n02x5               g192(.a(new_n287), .b(new_n284), .c(new_n214), .d(new_n194), .o1(new_n288));
  xorb03aa1n02x5               g193(.a(new_n288), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g194(.a(\b[24] ), .b(\a[25] ), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  nanp02aa1n03x5               g196(.a(new_n242), .b(new_n282), .o1(new_n292));
  nona22aa1n03x5               g197(.a(new_n292), .b(new_n286), .c(new_n285), .out0(new_n293));
  xorc02aa1n02x5               g198(.a(\a[25] ), .b(\b[24] ), .out0(new_n294));
  aoai13aa1n02x5               g199(.a(new_n294), .b(new_n293), .c(new_n195), .d(new_n283), .o1(new_n295));
  xorc02aa1n12x5               g200(.a(\a[26] ), .b(\b[25] ), .out0(new_n296));
  inv000aa1d42x5               g201(.a(new_n296), .o1(new_n297));
  aoi012aa1n03x5               g202(.a(new_n297), .b(new_n295), .c(new_n291), .o1(new_n298));
  aoi112aa1n02x5               g203(.a(new_n290), .b(new_n296), .c(new_n288), .d(new_n294), .o1(new_n299));
  norp02aa1n02x5               g204(.a(new_n298), .b(new_n299), .o1(\s[26] ));
  and002aa1n18x5               g205(.a(new_n296), .b(new_n294), .o(new_n301));
  nano22aa1n03x7               g206(.a(new_n240), .b(new_n282), .c(new_n301), .out0(new_n302));
  aoai13aa1n06x5               g207(.a(new_n302), .b(new_n207), .c(new_n204), .d(new_n206), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[26] ), .b(\b[25] ), .c(new_n291), .carry(new_n304));
  aobi12aa1n06x5               g209(.a(new_n304), .b(new_n293), .c(new_n301), .out0(new_n305));
  xorc02aa1n12x5               g210(.a(\a[27] ), .b(\b[26] ), .out0(new_n306));
  xnbna2aa1n03x5               g211(.a(new_n306), .b(new_n303), .c(new_n305), .out0(\s[27] ));
  norp02aa1n02x5               g212(.a(\b[26] ), .b(\a[27] ), .o1(new_n308));
  inv040aa1n03x5               g213(.a(new_n308), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n301), .o1(new_n310));
  oai012aa1n09x5               g215(.a(new_n304), .b(new_n287), .c(new_n310), .o1(new_n311));
  aoai13aa1n06x5               g216(.a(new_n306), .b(new_n311), .c(new_n195), .d(new_n302), .o1(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[27] ), .b(\a[28] ), .out0(new_n313));
  aoi012aa1n03x5               g218(.a(new_n313), .b(new_n312), .c(new_n309), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n306), .o1(new_n315));
  tech160nm_fiaoi012aa1n02p5x5 g220(.a(new_n315), .b(new_n303), .c(new_n305), .o1(new_n316));
  nano22aa1n03x7               g221(.a(new_n316), .b(new_n309), .c(new_n313), .out0(new_n317));
  norp02aa1n03x5               g222(.a(new_n314), .b(new_n317), .o1(\s[28] ));
  norb02aa1n02x5               g223(.a(new_n306), .b(new_n313), .out0(new_n319));
  aoai13aa1n06x5               g224(.a(new_n319), .b(new_n311), .c(new_n195), .d(new_n302), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[28] ), .b(\b[27] ), .c(new_n309), .carry(new_n321));
  xnrc02aa1n02x5               g226(.a(\b[28] ), .b(\a[29] ), .out0(new_n322));
  tech160nm_fiaoi012aa1n05x5   g227(.a(new_n322), .b(new_n320), .c(new_n321), .o1(new_n323));
  inv000aa1d42x5               g228(.a(new_n319), .o1(new_n324));
  tech160nm_fiaoi012aa1n02p5x5 g229(.a(new_n324), .b(new_n303), .c(new_n305), .o1(new_n325));
  nano22aa1n02x5               g230(.a(new_n325), .b(new_n321), .c(new_n322), .out0(new_n326));
  nor002aa1n02x5               g231(.a(new_n323), .b(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g232(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g233(.a(new_n306), .b(new_n322), .c(new_n313), .out0(new_n329));
  aoai13aa1n06x5               g234(.a(new_n329), .b(new_n311), .c(new_n195), .d(new_n302), .o1(new_n330));
  oao003aa1n02x5               g235(.a(\a[29] ), .b(\b[28] ), .c(new_n321), .carry(new_n331));
  xnrc02aa1n02x5               g236(.a(\b[29] ), .b(\a[30] ), .out0(new_n332));
  aoi012aa1n03x5               g237(.a(new_n332), .b(new_n330), .c(new_n331), .o1(new_n333));
  inv000aa1d42x5               g238(.a(new_n329), .o1(new_n334));
  tech160nm_fiaoi012aa1n02p5x5 g239(.a(new_n334), .b(new_n303), .c(new_n305), .o1(new_n335));
  nano22aa1n03x7               g240(.a(new_n335), .b(new_n331), .c(new_n332), .out0(new_n336));
  norp02aa1n03x5               g241(.a(new_n333), .b(new_n336), .o1(\s[30] ));
  norb02aa1d21x5               g242(.a(new_n329), .b(new_n332), .out0(new_n338));
  inv000aa1d42x5               g243(.a(new_n338), .o1(new_n339));
  aoi012aa1n02x5               g244(.a(new_n339), .b(new_n303), .c(new_n305), .o1(new_n340));
  oao003aa1n02x5               g245(.a(\a[30] ), .b(\b[29] ), .c(new_n331), .carry(new_n341));
  xnrc02aa1n02x5               g246(.a(\b[30] ), .b(\a[31] ), .out0(new_n342));
  nano22aa1n03x7               g247(.a(new_n340), .b(new_n341), .c(new_n342), .out0(new_n343));
  aoai13aa1n06x5               g248(.a(new_n338), .b(new_n311), .c(new_n195), .d(new_n302), .o1(new_n344));
  aoi012aa1n06x5               g249(.a(new_n342), .b(new_n344), .c(new_n341), .o1(new_n345));
  nor002aa1n02x5               g250(.a(new_n345), .b(new_n343), .o1(\s[31] ));
  xnrb03aa1n02x5               g251(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oai012aa1n02x5               g252(.a(new_n104), .b(\b[2] ), .c(\a[3] ), .o1(new_n348));
  oab012aa1n02x4               g253(.a(new_n348), .b(new_n103), .c(new_n105), .out0(new_n349));
  oaoi13aa1n02x5               g254(.a(new_n349), .b(new_n110), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorb03aa1n02x5               g255(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g256(.a(new_n115), .b(new_n110), .c(new_n116), .o1(new_n352));
  xnrb03aa1n02x5               g257(.a(new_n352), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g258(.a(\a[6] ), .b(\b[5] ), .c(new_n352), .o1(new_n354));
  xorb03aa1n02x5               g259(.a(new_n354), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  tech160nm_fiaoi012aa1n05x5   g260(.a(new_n118), .b(new_n354), .c(new_n120), .o1(new_n356));
  xnbna2aa1n03x5               g261(.a(new_n356), .b(new_n111), .c(new_n112), .out0(\s[8] ));
  xnbna2aa1n03x5               g262(.a(new_n128), .b(new_n149), .c(new_n126), .out0(\s[9] ));
endmodule


