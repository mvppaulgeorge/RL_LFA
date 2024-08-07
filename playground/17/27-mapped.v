// Benchmark "adder" written by ABC on Wed Jul 17 20:54:19 2024

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
    new_n139, new_n140, new_n141, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n172, new_n173, new_n174, new_n175, new_n176, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n187, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n255, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n294, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n342, new_n344, new_n346,
    new_n347, new_n348, new_n349, new_n350, new_n352, new_n353, new_n354,
    new_n356, new_n357, new_n360, new_n361;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n20x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nanp02aa1n24x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nor042aa1n06x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n09x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nona22aa1n03x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .out0(new_n101));
  nor042aa1n09x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand42aa1n08x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nano22aa1n03x7               g008(.a(new_n102), .b(new_n98), .c(new_n103), .out0(new_n104));
  nand42aa1n08x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  orn002aa1n24x5               g010(.a(\a[4] ), .b(\b[3] ), .o(new_n106));
  oai112aa1n06x5               g011(.a(new_n106), .b(new_n105), .c(\b[2] ), .d(\a[3] ), .o1(new_n107));
  tech160nm_fiaoi012aa1n05x5   g012(.a(new_n107), .b(new_n104), .c(new_n101), .o1(new_n108));
  nand02aa1n10x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand42aa1n10x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nano22aa1n03x7               g016(.a(new_n110), .b(new_n109), .c(new_n111), .out0(new_n112));
  aob012aa1n02x5               g017(.a(new_n105), .b(\b[4] ), .c(\a[5] ), .out0(new_n113));
  oai022aa1d24x5               g018(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n114));
  nor042aa1n06x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nand02aa1d16x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  norb02aa1n03x5               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  nona23aa1n03x5               g022(.a(new_n112), .b(new_n117), .c(new_n114), .d(new_n113), .out0(new_n118));
  inv040aa1n08x5               g023(.a(new_n110), .o1(new_n119));
  oaoi03aa1n12x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n119), .o1(new_n120));
  aoi013aa1n03x5               g025(.a(new_n120), .b(new_n112), .c(new_n114), .d(new_n117), .o1(new_n121));
  oai012aa1n09x5               g026(.a(new_n121), .b(new_n108), .c(new_n118), .o1(new_n122));
  nand42aa1d28x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  aoi012aa1n02x5               g028(.a(new_n97), .b(new_n122), .c(new_n123), .o1(new_n124));
  nor002aa1n20x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand42aa1d28x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  norb03aa1n09x5               g032(.a(new_n98), .b(new_n100), .c(new_n99), .out0(new_n128));
  nanb03aa1d24x5               g033(.a(new_n102), .b(new_n103), .c(new_n98), .out0(new_n129));
  oabi12aa1n18x5               g034(.a(new_n107), .b(new_n128), .c(new_n129), .out0(new_n130));
  nanb03aa1n06x5               g035(.a(new_n110), .b(new_n111), .c(new_n109), .out0(new_n131));
  aoi022aa1n06x5               g036(.a(\b[4] ), .b(\a[5] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n132));
  inv040aa1n02x5               g037(.a(new_n114), .o1(new_n133));
  nanb02aa1n02x5               g038(.a(new_n115), .b(new_n116), .out0(new_n134));
  nano23aa1n06x5               g039(.a(new_n131), .b(new_n134), .c(new_n133), .d(new_n132), .out0(new_n135));
  inv000aa1n02x5               g040(.a(new_n120), .o1(new_n136));
  oai013aa1n03x5               g041(.a(new_n136), .b(new_n131), .c(new_n134), .d(new_n133), .o1(new_n137));
  aoi012aa1n12x5               g042(.a(new_n137), .b(new_n135), .c(new_n130), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n123), .b(new_n97), .out0(new_n139));
  norb03aa1n02x5               g044(.a(new_n126), .b(new_n97), .c(new_n125), .out0(new_n140));
  oaib12aa1n02x5               g045(.a(new_n140), .b(new_n138), .c(new_n139), .out0(new_n141));
  oai012aa1n02x5               g046(.a(new_n141), .b(new_n124), .c(new_n127), .o1(\s[10] ));
  nano23aa1d15x5               g047(.a(new_n97), .b(new_n125), .c(new_n126), .d(new_n123), .out0(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n137), .c(new_n135), .d(new_n130), .o1(new_n144));
  oai012aa1n02x5               g049(.a(new_n126), .b(new_n125), .c(new_n97), .o1(new_n145));
  xorc02aa1n12x5               g050(.a(\a[11] ), .b(\b[10] ), .out0(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n146), .b(new_n144), .c(new_n145), .out0(\s[11] ));
  orn002aa1n24x5               g052(.a(\a[11] ), .b(\b[10] ), .o(new_n148));
  aob012aa1n02x5               g053(.a(new_n146), .b(new_n144), .c(new_n145), .out0(new_n149));
  nor042aa1n04x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  and002aa1n12x5               g055(.a(\b[11] ), .b(\a[12] ), .o(new_n151));
  nor002aa1n04x5               g056(.a(new_n151), .b(new_n150), .o1(new_n152));
  norb03aa1n02x5               g057(.a(new_n148), .b(new_n151), .c(new_n150), .out0(new_n153));
  nanp02aa1n02x5               g058(.a(new_n149), .b(new_n153), .o1(new_n154));
  aoai13aa1n02x5               g059(.a(new_n154), .b(new_n152), .c(new_n149), .d(new_n148), .o1(\s[12] ));
  nanp03aa1n09x5               g060(.a(new_n143), .b(new_n146), .c(new_n152), .o1(new_n156));
  inv040aa1n02x5               g061(.a(new_n156), .o1(new_n157));
  aoi112aa1n06x5               g062(.a(new_n151), .b(new_n150), .c(\a[11] ), .d(\b[10] ), .o1(new_n158));
  tech160nm_fioai012aa1n05x5   g063(.a(new_n126), .b(\b[10] ), .c(\a[11] ), .o1(new_n159));
  oab012aa1n09x5               g064(.a(new_n159), .b(new_n97), .c(new_n125), .out0(new_n160));
  nanp02aa1n06x5               g065(.a(new_n160), .b(new_n158), .o1(new_n161));
  oaoi03aa1n09x5               g066(.a(\a[12] ), .b(\b[11] ), .c(new_n148), .o1(new_n162));
  inv020aa1n03x5               g067(.a(new_n162), .o1(new_n163));
  nand02aa1d04x5               g068(.a(new_n161), .b(new_n163), .o1(new_n164));
  nor002aa1d32x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nand22aa1n04x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  aoai13aa1n06x5               g072(.a(new_n167), .b(new_n164), .c(new_n122), .d(new_n157), .o1(new_n168));
  nona22aa1n02x4               g073(.a(new_n161), .b(new_n162), .c(new_n167), .out0(new_n169));
  aoi012aa1n02x5               g074(.a(new_n169), .b(new_n122), .c(new_n157), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n168), .b(new_n170), .out0(\s[13] ));
  inv000aa1d42x5               g076(.a(new_n165), .o1(new_n172));
  nor002aa1d32x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nanp02aa1n12x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  nona23aa1n02x4               g080(.a(new_n168), .b(new_n174), .c(new_n173), .d(new_n165), .out0(new_n176));
  aoai13aa1n02x5               g081(.a(new_n176), .b(new_n175), .c(new_n172), .d(new_n168), .o1(\s[14] ));
  nona23aa1n03x5               g082(.a(new_n174), .b(new_n166), .c(new_n165), .d(new_n173), .out0(new_n178));
  nona22aa1n09x5               g083(.a(new_n122), .b(new_n156), .c(new_n178), .out0(new_n179));
  nano23aa1n06x5               g084(.a(new_n165), .b(new_n173), .c(new_n174), .d(new_n166), .out0(new_n180));
  aoai13aa1n06x5               g085(.a(new_n180), .b(new_n162), .c(new_n160), .d(new_n158), .o1(new_n181));
  tech160nm_fioai012aa1n03p5x5 g086(.a(new_n174), .b(new_n173), .c(new_n165), .o1(new_n182));
  xnrc02aa1n12x5               g087(.a(\b[14] ), .b(\a[15] ), .out0(new_n183));
  aoi013aa1n06x4               g088(.a(new_n183), .b(new_n179), .c(new_n181), .d(new_n182), .o1(new_n184));
  inv000aa1n02x5               g089(.a(new_n182), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n183), .o1(new_n186));
  aoi112aa1n02x5               g091(.a(new_n186), .b(new_n185), .c(new_n164), .d(new_n180), .o1(new_n187));
  aoi012aa1n02x5               g092(.a(new_n184), .b(new_n179), .c(new_n187), .o1(\s[15] ));
  nor002aa1n02x5               g093(.a(\b[14] ), .b(\a[15] ), .o1(new_n189));
  tech160nm_fixnrc02aa1n02p5x5 g094(.a(\b[15] ), .b(\a[16] ), .out0(new_n190));
  oai012aa1n02x5               g095(.a(new_n190), .b(new_n184), .c(new_n189), .o1(new_n191));
  oai022aa1n02x5               g096(.a(\a[15] ), .b(\b[14] ), .c(\b[15] ), .d(\a[16] ), .o1(new_n192));
  aoi012aa1n02x5               g097(.a(new_n192), .b(\a[16] ), .c(\b[15] ), .o1(new_n193));
  oaib12aa1n02x5               g098(.a(new_n191), .b(new_n184), .c(new_n193), .out0(\s[16] ));
  nor042aa1n03x5               g099(.a(new_n190), .b(new_n183), .o1(new_n195));
  nano22aa1n03x7               g100(.a(new_n156), .b(new_n195), .c(new_n180), .out0(new_n196));
  norb02aa1n06x4               g101(.a(new_n196), .b(new_n138), .out0(new_n197));
  nona32aa1n03x5               g102(.a(new_n157), .b(new_n190), .c(new_n183), .d(new_n178), .out0(new_n198));
  aoai13aa1n04x5               g103(.a(new_n195), .b(new_n185), .c(new_n164), .d(new_n180), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\a[16] ), .o1(new_n200));
  inv000aa1d42x5               g105(.a(\b[15] ), .o1(new_n201));
  oao003aa1n02x5               g106(.a(new_n200), .b(new_n201), .c(new_n189), .carry(new_n202));
  inv000aa1n02x5               g107(.a(new_n202), .o1(new_n203));
  oai112aa1n06x5               g108(.a(new_n199), .b(new_n203), .c(new_n198), .d(new_n138), .o1(new_n204));
  xorc02aa1n12x5               g109(.a(\a[17] ), .b(\b[16] ), .out0(new_n205));
  inv030aa1n02x5               g110(.a(new_n195), .o1(new_n206));
  tech160nm_fiaoi012aa1n05x5   g111(.a(new_n206), .b(new_n181), .c(new_n182), .o1(new_n207));
  norp03aa1n02x5               g112(.a(new_n207), .b(new_n202), .c(new_n205), .o1(new_n208));
  aboi22aa1n03x5               g113(.a(new_n197), .b(new_n208), .c(new_n204), .d(new_n205), .out0(\s[17] ));
  aoai13aa1n06x5               g114(.a(new_n203), .b(new_n206), .c(new_n181), .d(new_n182), .o1(new_n210));
  norp02aa1n09x5               g115(.a(\b[16] ), .b(\a[17] ), .o1(new_n211));
  oaoi13aa1n02x5               g116(.a(new_n211), .b(new_n205), .c(new_n197), .d(new_n210), .o1(new_n212));
  norp02aa1n24x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  nand02aa1d08x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  norb02aa1n03x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  aoi112aa1n06x5               g120(.a(new_n207), .b(new_n202), .c(new_n122), .d(new_n196), .o1(new_n216));
  norb03aa1n02x5               g121(.a(new_n214), .b(new_n211), .c(new_n213), .out0(new_n217));
  oaib12aa1n02x5               g122(.a(new_n217), .b(new_n216), .c(new_n205), .out0(new_n218));
  oai012aa1n02x5               g123(.a(new_n218), .b(new_n212), .c(new_n215), .o1(\s[18] ));
  and002aa1n06x5               g124(.a(new_n205), .b(new_n215), .o(new_n220));
  oa0012aa1n02x5               g125(.a(new_n214), .b(new_n213), .c(new_n211), .o(new_n221));
  xorc02aa1n12x5               g126(.a(\a[19] ), .b(\b[18] ), .out0(new_n222));
  aoai13aa1n06x5               g127(.a(new_n222), .b(new_n221), .c(new_n204), .d(new_n220), .o1(new_n223));
  aoi112aa1n02x7               g128(.a(new_n222), .b(new_n221), .c(new_n204), .d(new_n220), .o1(new_n224));
  norb02aa1n03x4               g129(.a(new_n223), .b(new_n224), .out0(\s[19] ));
  xnrc02aa1n02x5               g130(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1d32x5               g131(.a(\a[19] ), .o1(new_n227));
  inv040aa1d32x5               g132(.a(\b[18] ), .o1(new_n228));
  nand22aa1n04x5               g133(.a(new_n228), .b(new_n227), .o1(new_n229));
  nor002aa1n03x5               g134(.a(\b[19] ), .b(\a[20] ), .o1(new_n230));
  nand42aa1d28x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n231), .b(new_n230), .out0(new_n232));
  nano22aa1n02x4               g137(.a(new_n230), .b(new_n229), .c(new_n231), .out0(new_n233));
  nanp02aa1n03x5               g138(.a(new_n223), .b(new_n233), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n234), .b(new_n232), .c(new_n223), .d(new_n229), .o1(\s[20] ));
  and003aa1n06x5               g140(.a(new_n220), .b(new_n232), .c(new_n222), .o(new_n236));
  nand42aa1n03x5               g141(.a(\b[18] ), .b(\a[19] ), .o1(new_n237));
  nano22aa1n03x7               g142(.a(new_n230), .b(new_n237), .c(new_n231), .out0(new_n238));
  tech160nm_fioai012aa1n05x5   g143(.a(new_n214), .b(\b[18] ), .c(\a[19] ), .o1(new_n239));
  oab012aa1n06x5               g144(.a(new_n239), .b(new_n211), .c(new_n213), .out0(new_n240));
  oaoi03aa1n03x5               g145(.a(\a[20] ), .b(\b[19] ), .c(new_n229), .o1(new_n241));
  tech160nm_fiao0012aa1n02p5x5 g146(.a(new_n241), .b(new_n240), .c(new_n238), .o(new_n242));
  nor002aa1n16x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  nand42aa1n02x5               g148(.a(\b[20] ), .b(\a[21] ), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n242), .c(new_n204), .d(new_n236), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(new_n241), .b(new_n245), .c(new_n240), .d(new_n238), .o1(new_n247));
  aobi12aa1n02x5               g152(.a(new_n247), .b(new_n204), .c(new_n236), .out0(new_n248));
  norb02aa1n03x4               g153(.a(new_n246), .b(new_n248), .out0(\s[21] ));
  inv000aa1d42x5               g154(.a(new_n243), .o1(new_n250));
  nor042aa1n02x5               g155(.a(\b[21] ), .b(\a[22] ), .o1(new_n251));
  nanp02aa1n04x5               g156(.a(\b[21] ), .b(\a[22] ), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n252), .b(new_n251), .out0(new_n253));
  norb03aa1n02x5               g158(.a(new_n252), .b(new_n243), .c(new_n251), .out0(new_n254));
  nanp02aa1n03x5               g159(.a(new_n246), .b(new_n254), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n255), .b(new_n253), .c(new_n246), .d(new_n250), .o1(\s[22] ));
  nano23aa1n06x5               g161(.a(new_n243), .b(new_n251), .c(new_n252), .d(new_n244), .out0(new_n257));
  inv020aa1n03x5               g162(.a(new_n257), .o1(new_n258));
  nano32aa1n02x4               g163(.a(new_n258), .b(new_n220), .c(new_n222), .d(new_n232), .out0(new_n259));
  aoai13aa1n06x5               g164(.a(new_n257), .b(new_n241), .c(new_n240), .d(new_n238), .o1(new_n260));
  oai012aa1n02x5               g165(.a(new_n252), .b(new_n251), .c(new_n243), .o1(new_n261));
  nanp02aa1n02x5               g166(.a(new_n260), .b(new_n261), .o1(new_n262));
  xorc02aa1n12x5               g167(.a(\a[23] ), .b(\b[22] ), .out0(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n262), .c(new_n204), .d(new_n259), .o1(new_n264));
  nanb02aa1n02x5               g169(.a(new_n263), .b(new_n261), .out0(new_n265));
  norb02aa1n02x5               g170(.a(new_n260), .b(new_n265), .out0(new_n266));
  aobi12aa1n02x5               g171(.a(new_n266), .b(new_n204), .c(new_n259), .out0(new_n267));
  norb02aa1n03x4               g172(.a(new_n264), .b(new_n267), .out0(\s[23] ));
  norp02aa1n02x5               g173(.a(\b[22] ), .b(\a[23] ), .o1(new_n269));
  inv000aa1d42x5               g174(.a(new_n269), .o1(new_n270));
  xorc02aa1n12x5               g175(.a(\a[24] ), .b(\b[23] ), .out0(new_n271));
  oai022aa1n02x5               g176(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n272));
  aoi012aa1n02x5               g177(.a(new_n272), .b(\a[24] ), .c(\b[23] ), .o1(new_n273));
  nanp02aa1n03x5               g178(.a(new_n264), .b(new_n273), .o1(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n271), .c(new_n264), .d(new_n270), .o1(\s[24] ));
  nano22aa1n02x4               g180(.a(new_n258), .b(new_n263), .c(new_n271), .out0(new_n276));
  and002aa1n02x5               g181(.a(new_n236), .b(new_n276), .o(new_n277));
  and002aa1n02x7               g182(.a(new_n271), .b(new_n263), .o(new_n278));
  inv000aa1n02x5               g183(.a(new_n278), .o1(new_n279));
  aob012aa1n02x5               g184(.a(new_n272), .b(\b[23] ), .c(\a[24] ), .out0(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n279), .c(new_n260), .d(new_n261), .o1(new_n281));
  xorc02aa1n02x5               g186(.a(\a[25] ), .b(\b[24] ), .out0(new_n282));
  aoai13aa1n06x5               g187(.a(new_n282), .b(new_n281), .c(new_n204), .d(new_n277), .o1(new_n283));
  nanb02aa1n02x5               g188(.a(new_n282), .b(new_n280), .out0(new_n284));
  aoi012aa1n02x5               g189(.a(new_n284), .b(new_n262), .c(new_n278), .o1(new_n285));
  aobi12aa1n02x5               g190(.a(new_n285), .b(new_n204), .c(new_n277), .out0(new_n286));
  norb02aa1n03x4               g191(.a(new_n283), .b(new_n286), .out0(\s[25] ));
  norp02aa1n02x5               g192(.a(\b[24] ), .b(\a[25] ), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n288), .o1(new_n289));
  xorc02aa1n02x5               g194(.a(\a[26] ), .b(\b[25] ), .out0(new_n290));
  nanp02aa1n02x5               g195(.a(\b[25] ), .b(\a[26] ), .o1(new_n291));
  oai022aa1n02x5               g196(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n292));
  norb02aa1n02x5               g197(.a(new_n291), .b(new_n292), .out0(new_n293));
  nanp02aa1n03x5               g198(.a(new_n283), .b(new_n293), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n294), .b(new_n290), .c(new_n283), .d(new_n289), .o1(\s[26] ));
  and002aa1n02x5               g200(.a(new_n290), .b(new_n282), .o(new_n296));
  nand23aa1n03x5               g201(.a(new_n236), .b(new_n276), .c(new_n296), .o1(new_n297));
  inv040aa1n03x5               g202(.a(new_n297), .o1(new_n298));
  oai012aa1n06x5               g203(.a(new_n298), .b(new_n197), .c(new_n210), .o1(new_n299));
  aoi022aa1n09x5               g204(.a(new_n281), .b(new_n296), .c(new_n291), .d(new_n292), .o1(new_n300));
  tech160nm_fioai012aa1n05x5   g205(.a(new_n300), .b(new_n216), .c(new_n297), .o1(new_n301));
  xorc02aa1n12x5               g206(.a(\a[27] ), .b(\b[26] ), .out0(new_n302));
  aoi122aa1n02x5               g207(.a(new_n302), .b(new_n291), .c(new_n292), .d(new_n281), .e(new_n296), .o1(new_n303));
  aoi022aa1n02x5               g208(.a(new_n301), .b(new_n302), .c(new_n303), .d(new_n299), .o1(\s[27] ));
  norp02aa1n02x5               g209(.a(\b[26] ), .b(\a[27] ), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n305), .o1(new_n306));
  nanp02aa1n03x5               g211(.a(new_n301), .b(new_n302), .o1(new_n307));
  xorc02aa1n02x5               g212(.a(\a[28] ), .b(\b[27] ), .out0(new_n308));
  inv000aa1d42x5               g213(.a(new_n302), .o1(new_n309));
  oai022aa1d24x5               g214(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n310));
  aoi012aa1n02x5               g215(.a(new_n310), .b(\a[28] ), .c(\b[27] ), .o1(new_n311));
  aoai13aa1n02x7               g216(.a(new_n311), .b(new_n309), .c(new_n299), .d(new_n300), .o1(new_n312));
  aoai13aa1n03x5               g217(.a(new_n312), .b(new_n308), .c(new_n307), .d(new_n306), .o1(\s[28] ));
  and002aa1n06x5               g218(.a(new_n308), .b(new_n302), .o(new_n314));
  inv000aa1n02x5               g219(.a(new_n314), .o1(new_n315));
  inv000aa1d42x5               g220(.a(\b[27] ), .o1(new_n316));
  oaib12aa1n18x5               g221(.a(new_n310), .b(new_n316), .c(\a[28] ), .out0(new_n317));
  inv000aa1d42x5               g222(.a(new_n317), .o1(new_n318));
  tech160nm_fixorc02aa1n03p5x5 g223(.a(\a[29] ), .b(\b[28] ), .out0(new_n319));
  norb02aa1n02x5               g224(.a(new_n319), .b(new_n318), .out0(new_n320));
  aoai13aa1n02x7               g225(.a(new_n320), .b(new_n315), .c(new_n299), .d(new_n300), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n319), .o1(new_n322));
  aoai13aa1n03x5               g227(.a(new_n322), .b(new_n318), .c(new_n301), .d(new_n314), .o1(new_n323));
  nanp02aa1n03x5               g228(.a(new_n323), .b(new_n321), .o1(\s[29] ));
  xorb03aa1n02x5               g229(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g230(.a(new_n322), .b(new_n302), .c(new_n308), .out0(new_n326));
  tech160nm_fioaoi03aa1n03p5x5 g231(.a(\a[29] ), .b(\b[28] ), .c(new_n317), .o1(new_n327));
  xnrc02aa1n02x5               g232(.a(\b[29] ), .b(\a[30] ), .out0(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n327), .c(new_n301), .d(new_n326), .o1(new_n329));
  inv000aa1d42x5               g234(.a(new_n326), .o1(new_n330));
  norp02aa1n03x5               g235(.a(new_n327), .b(new_n328), .o1(new_n331));
  aoai13aa1n02x5               g236(.a(new_n331), .b(new_n330), .c(new_n299), .d(new_n300), .o1(new_n332));
  nanp02aa1n03x5               g237(.a(new_n329), .b(new_n332), .o1(\s[30] ));
  nano32aa1n03x7               g238(.a(new_n328), .b(new_n319), .c(new_n308), .d(new_n302), .out0(new_n334));
  aoi012aa1n02x5               g239(.a(new_n331), .b(\a[30] ), .c(\b[29] ), .o1(new_n335));
  xnrc02aa1n02x5               g240(.a(\b[30] ), .b(\a[31] ), .out0(new_n336));
  aoai13aa1n03x5               g241(.a(new_n336), .b(new_n335), .c(new_n301), .d(new_n334), .o1(new_n337));
  inv000aa1d42x5               g242(.a(new_n334), .o1(new_n338));
  norp02aa1n02x5               g243(.a(new_n335), .b(new_n336), .o1(new_n339));
  aoai13aa1n02x7               g244(.a(new_n339), .b(new_n338), .c(new_n299), .d(new_n300), .o1(new_n340));
  nanp02aa1n03x5               g245(.a(new_n337), .b(new_n340), .o1(\s[31] ));
  aboi22aa1n03x5               g246(.a(new_n102), .b(new_n103), .c(new_n101), .d(new_n98), .out0(new_n342));
  aoi012aa1n02x5               g247(.a(new_n342), .b(new_n101), .c(new_n104), .o1(\s[3] ));
  aoi012aa1n02x5               g248(.a(new_n102), .b(new_n104), .c(new_n101), .o1(new_n344));
  aoai13aa1n02x5               g249(.a(new_n130), .b(new_n344), .c(new_n106), .d(new_n105), .o1(\s[4] ));
  inv000aa1d42x5               g250(.a(\a[5] ), .o1(new_n346));
  inv000aa1d42x5               g251(.a(\b[4] ), .o1(new_n347));
  nanp02aa1n02x5               g252(.a(new_n347), .b(new_n346), .o1(new_n348));
  nanp02aa1n02x5               g253(.a(\b[4] ), .b(\a[5] ), .o1(new_n349));
  aoi022aa1n02x5               g254(.a(new_n130), .b(new_n105), .c(new_n348), .d(new_n349), .o1(new_n350));
  aoi013aa1n02x4               g255(.a(new_n350), .b(new_n348), .c(new_n132), .d(new_n130), .o1(\s[5] ));
  nanp03aa1n02x5               g256(.a(new_n130), .b(new_n132), .c(new_n348), .o1(new_n352));
  xorc02aa1n02x5               g257(.a(\a[6] ), .b(\b[5] ), .out0(new_n353));
  nanp03aa1n02x5               g258(.a(new_n352), .b(new_n109), .c(new_n133), .o1(new_n354));
  aoai13aa1n02x5               g259(.a(new_n354), .b(new_n353), .c(new_n348), .d(new_n352), .o1(\s[6] ));
  nanp02aa1n02x5               g260(.a(new_n354), .b(new_n112), .o1(new_n356));
  aoi022aa1n02x5               g261(.a(new_n354), .b(new_n109), .c(new_n119), .d(new_n111), .o1(new_n357));
  norb02aa1n02x5               g262(.a(new_n356), .b(new_n357), .out0(\s[7] ));
  xnbna2aa1n03x5               g263(.a(new_n117), .b(new_n356), .c(new_n119), .out0(\s[8] ));
  nanp02aa1n02x5               g264(.a(new_n135), .b(new_n130), .o1(new_n360));
  aoi113aa1n02x5               g265(.a(new_n120), .b(new_n139), .c(new_n112), .d(new_n114), .e(new_n117), .o1(new_n361));
  aoi022aa1n02x5               g266(.a(new_n122), .b(new_n139), .c(new_n360), .d(new_n361), .o1(\s[9] ));
endmodule


