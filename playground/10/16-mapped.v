// Benchmark "adder" written by ABC on Wed Jul 17 17:11:38 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n313, new_n315, new_n317, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  and002aa1n02x5               g003(.a(\b[5] ), .b(\a[6] ), .o(new_n99));
  inv000aa1d42x5               g004(.a(\a[5] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\a[6] ), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\b[4] ), .o1(new_n102));
  aboi22aa1n03x5               g007(.a(\b[5] ), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n103));
  norp02aa1n06x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  nand02aa1d04x5               g009(.a(\b[7] ), .b(\a[8] ), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  nand42aa1n02x5               g011(.a(\b[6] ), .b(\a[7] ), .o1(new_n107));
  nona23aa1n09x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  oai012aa1n02x5               g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  oai013aa1n03x5               g014(.a(new_n109), .b(new_n108), .c(new_n99), .d(new_n103), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  nand22aa1n03x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nor002aa1n02x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  tech160nm_fioai012aa1n04x5   g018(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n114));
  nor002aa1n02x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nor022aa1n08x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nona23aa1n06x5               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  oai012aa1n02x5               g024(.a(new_n116), .b(new_n117), .c(new_n115), .o1(new_n120));
  oaih12aa1n06x5               g025(.a(new_n120), .b(new_n119), .c(new_n114), .o1(new_n121));
  xnrc02aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .out0(new_n122));
  xnrc02aa1n02x5               g027(.a(\b[4] ), .b(\a[5] ), .out0(new_n123));
  nor043aa1n02x5               g028(.a(new_n108), .b(new_n122), .c(new_n123), .o1(new_n124));
  xorc02aa1n02x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n03x5               g030(.a(new_n125), .b(new_n110), .c(new_n124), .d(new_n121), .o1(new_n126));
  xorc02aa1n12x5               g031(.a(\a[10] ), .b(\b[9] ), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g033(.a(new_n127), .o1(new_n129));
  norp02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  aoi112aa1n09x5               g035(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n131));
  norp02aa1n02x5               g036(.a(new_n131), .b(new_n130), .o1(new_n132));
  aoai13aa1n06x5               g037(.a(new_n132), .b(new_n129), .c(new_n126), .d(new_n98), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nand42aa1n20x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nor042aa1n04x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nor022aa1n06x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand42aa1n08x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  aoi112aa1n02x5               g044(.a(new_n139), .b(new_n136), .c(new_n133), .d(new_n135), .o1(new_n140));
  aoai13aa1n02x5               g045(.a(new_n139), .b(new_n136), .c(new_n133), .d(new_n135), .o1(new_n141));
  norb02aa1n03x4               g046(.a(new_n141), .b(new_n140), .out0(\s[12] ));
  nano23aa1d15x5               g047(.a(new_n137), .b(new_n136), .c(new_n138), .d(new_n135), .out0(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  nano22aa1n02x4               g049(.a(new_n144), .b(new_n125), .c(new_n127), .out0(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n110), .c(new_n124), .d(new_n121), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n135), .b(new_n136), .out0(new_n147));
  oai112aa1n03x5               g052(.a(new_n139), .b(new_n147), .c(new_n131), .d(new_n130), .o1(new_n148));
  oai012aa1n02x5               g053(.a(new_n138), .b(new_n137), .c(new_n136), .o1(new_n149));
  and002aa1n02x5               g054(.a(new_n148), .b(new_n149), .o(new_n150));
  nanp02aa1n03x5               g055(.a(new_n146), .b(new_n150), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nand42aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norp02aa1n04x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n154), .b(new_n151), .c(new_n153), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  tech160nm_fixorc02aa1n02p5x5 g061(.a(\a[15] ), .b(\b[14] ), .out0(new_n157));
  nor002aa1n03x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nano23aa1n06x5               g064(.a(new_n158), .b(new_n154), .c(new_n159), .d(new_n153), .out0(new_n160));
  tech160nm_fiao0012aa1n02p5x5 g065(.a(new_n158), .b(new_n154), .c(new_n159), .o(new_n161));
  aoai13aa1n06x5               g066(.a(new_n157), .b(new_n161), .c(new_n151), .d(new_n160), .o1(new_n162));
  aoi112aa1n02x5               g067(.a(new_n157), .b(new_n161), .c(new_n151), .d(new_n160), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(\s[15] ));
  inv000aa1d42x5               g069(.a(\a[15] ), .o1(new_n165));
  inv000aa1d42x5               g070(.a(\b[14] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(new_n166), .b(new_n165), .o1(new_n167));
  xorc02aa1n12x5               g072(.a(\a[16] ), .b(\b[15] ), .out0(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  nanp03aa1n02x5               g074(.a(new_n162), .b(new_n167), .c(new_n169), .o1(new_n170));
  tech160nm_fiaoi012aa1n02p5x5 g075(.a(new_n169), .b(new_n162), .c(new_n167), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(\s[16] ));
  nand23aa1n03x5               g077(.a(new_n160), .b(new_n157), .c(new_n168), .o1(new_n173));
  nano32aa1n03x7               g078(.a(new_n173), .b(new_n143), .c(new_n127), .d(new_n125), .out0(new_n174));
  aoai13aa1n06x5               g079(.a(new_n174), .b(new_n110), .c(new_n124), .d(new_n121), .o1(new_n175));
  inv000aa1d42x5               g080(.a(\a[16] ), .o1(new_n176));
  inv000aa1d42x5               g081(.a(\b[15] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(new_n177), .b(new_n176), .o1(new_n178));
  oai112aa1n02x5               g083(.a(new_n165), .b(new_n166), .c(new_n177), .d(new_n176), .o1(new_n179));
  nanp03aa1n02x5               g084(.a(new_n161), .b(new_n157), .c(new_n168), .o1(new_n180));
  aoi012aa1n03x5               g085(.a(new_n173), .b(new_n148), .c(new_n149), .o1(new_n181));
  nano32aa1n03x7               g086(.a(new_n181), .b(new_n180), .c(new_n179), .d(new_n178), .out0(new_n182));
  nand02aa1d08x5               g087(.a(new_n175), .b(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g089(.a(\a[18] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\b[16] ), .o1(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  xroi22aa1d04x5               g094(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n190));
  nanp02aa1n02x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  nona22aa1n02x4               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(new_n192));
  oaib12aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n185), .out0(new_n193));
  nand42aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  nor042aa1n03x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n194), .b(new_n195), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n193), .c(new_n183), .d(new_n190), .o1(new_n197));
  aoi112aa1n02x5               g102(.a(new_n196), .b(new_n193), .c(new_n183), .d(new_n190), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n04x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  nona22aa1n03x5               g108(.a(new_n197), .b(new_n203), .c(new_n195), .out0(new_n204));
  orn002aa1n24x5               g109(.a(\a[19] ), .b(\b[18] ), .o(new_n205));
  aobi12aa1n06x5               g110(.a(new_n203), .b(new_n197), .c(new_n205), .out0(new_n206));
  norb02aa1n03x4               g111(.a(new_n204), .b(new_n206), .out0(\s[20] ));
  nano23aa1n06x5               g112(.a(new_n201), .b(new_n195), .c(new_n202), .d(new_n194), .out0(new_n208));
  nand22aa1n06x5               g113(.a(new_n190), .b(new_n208), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  norp02aa1n02x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  aoi013aa1n02x4               g116(.a(new_n211), .b(new_n191), .c(new_n186), .d(new_n187), .o1(new_n212));
  nona23aa1n02x4               g117(.a(new_n194), .b(new_n202), .c(new_n201), .d(new_n195), .out0(new_n213));
  oaoi03aa1n12x5               g118(.a(\a[20] ), .b(\b[19] ), .c(new_n205), .o1(new_n214));
  inv000aa1n02x5               g119(.a(new_n214), .o1(new_n215));
  oai012aa1n06x5               g120(.a(new_n215), .b(new_n213), .c(new_n212), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  aoai13aa1n06x5               g122(.a(new_n217), .b(new_n216), .c(new_n183), .d(new_n210), .o1(new_n218));
  aoi112aa1n02x5               g123(.a(new_n217), .b(new_n216), .c(new_n183), .d(new_n210), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n218), .b(new_n219), .out0(\s[21] ));
  nor042aa1n03x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  inv040aa1n03x5               g126(.a(new_n221), .o1(new_n222));
  tech160nm_fixnrc02aa1n04x5   g127(.a(\b[21] ), .b(\a[22] ), .out0(new_n223));
  nanp03aa1n03x5               g128(.a(new_n218), .b(new_n222), .c(new_n223), .o1(new_n224));
  tech160nm_fiaoi012aa1n02p5x5 g129(.a(new_n223), .b(new_n218), .c(new_n222), .o1(new_n225));
  norb02aa1n03x4               g130(.a(new_n224), .b(new_n225), .out0(\s[22] ));
  nanp02aa1n02x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  nano22aa1n03x7               g132(.a(new_n223), .b(new_n227), .c(new_n222), .out0(new_n228));
  and003aa1n02x5               g133(.a(new_n190), .b(new_n228), .c(new_n208), .o(new_n229));
  aoai13aa1n06x5               g134(.a(new_n228), .b(new_n214), .c(new_n208), .d(new_n193), .o1(new_n230));
  oaoi03aa1n09x5               g135(.a(\a[22] ), .b(\b[21] ), .c(new_n222), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  nanp02aa1n02x5               g137(.a(new_n230), .b(new_n232), .o1(new_n233));
  aoi012aa1n06x5               g138(.a(new_n233), .b(new_n183), .c(new_n229), .o1(new_n234));
  nor002aa1n06x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  inv000aa1n02x5               g140(.a(new_n235), .o1(new_n236));
  and002aa1n12x5               g141(.a(\b[22] ), .b(\a[23] ), .o(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  xnbna2aa1n03x5               g143(.a(new_n234), .b(new_n238), .c(new_n236), .out0(\s[23] ));
  xorc02aa1n02x5               g144(.a(\a[24] ), .b(\b[23] ), .out0(new_n240));
  nanp02aa1n03x5               g145(.a(new_n183), .b(new_n229), .o1(new_n241));
  nona22aa1n03x5               g146(.a(new_n241), .b(new_n233), .c(new_n235), .out0(new_n242));
  tech160nm_fiaoi012aa1n04x5   g147(.a(new_n240), .b(new_n242), .c(new_n238), .o1(new_n243));
  nand43aa1n02x5               g148(.a(new_n242), .b(new_n238), .c(new_n240), .o1(new_n244));
  norb02aa1n02x7               g149(.a(new_n244), .b(new_n243), .out0(\s[24] ));
  nor042aa1n02x5               g150(.a(\b[23] ), .b(\a[24] ), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(\b[23] ), .b(\a[24] ), .o1(new_n247));
  nano23aa1d15x5               g152(.a(new_n246), .b(new_n237), .c(new_n236), .d(new_n247), .out0(new_n248));
  nanb03aa1n03x5               g153(.a(new_n209), .b(new_n248), .c(new_n228), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n248), .o1(new_n250));
  oai012aa1n02x5               g155(.a(new_n247), .b(new_n246), .c(new_n235), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n250), .c(new_n230), .d(new_n232), .o1(new_n252));
  inv040aa1n03x5               g157(.a(new_n252), .o1(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n249), .c(new_n175), .d(new_n182), .o1(new_n254));
  xorb03aa1n02x5               g159(.a(new_n254), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  xorc02aa1n02x5               g161(.a(\a[25] ), .b(\b[24] ), .out0(new_n257));
  xorc02aa1n02x5               g162(.a(\a[26] ), .b(\b[25] ), .out0(new_n258));
  aoi112aa1n02x5               g163(.a(new_n256), .b(new_n258), .c(new_n254), .d(new_n257), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n258), .b(new_n256), .c(new_n254), .d(new_n257), .o1(new_n260));
  norb02aa1n02x7               g165(.a(new_n260), .b(new_n259), .out0(\s[26] ));
  nand22aa1n04x5               g166(.a(new_n258), .b(new_n257), .o1(new_n262));
  nano23aa1n06x5               g167(.a(new_n209), .b(new_n262), .c(new_n228), .d(new_n248), .out0(new_n263));
  nand02aa1d06x5               g168(.a(new_n183), .b(new_n263), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n262), .o1(new_n265));
  orn002aa1n02x5               g170(.a(\a[25] ), .b(\b[24] ), .o(new_n266));
  oao003aa1n02x5               g171(.a(\a[26] ), .b(\b[25] ), .c(new_n266), .carry(new_n267));
  aobi12aa1n06x5               g172(.a(new_n267), .b(new_n252), .c(new_n265), .out0(new_n268));
  nor002aa1n04x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  and002aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o(new_n270));
  norp02aa1n02x5               g175(.a(new_n270), .b(new_n269), .o1(new_n271));
  xnbna2aa1n03x5               g176(.a(new_n271), .b(new_n268), .c(new_n264), .out0(\s[27] ));
  inv000aa1d42x5               g177(.a(new_n270), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[28] ), .b(\b[27] ), .out0(new_n274));
  inv000aa1d42x5               g179(.a(new_n269), .o1(new_n275));
  nanp03aa1n03x5               g180(.a(new_n268), .b(new_n264), .c(new_n275), .o1(new_n276));
  tech160nm_fiaoi012aa1n05x5   g181(.a(new_n274), .b(new_n276), .c(new_n273), .o1(new_n277));
  aobi12aa1n06x5               g182(.a(new_n263), .b(new_n175), .c(new_n182), .out0(new_n278));
  aoai13aa1n02x5               g183(.a(new_n248), .b(new_n231), .c(new_n216), .d(new_n228), .o1(new_n279));
  aoai13aa1n04x5               g184(.a(new_n267), .b(new_n262), .c(new_n279), .d(new_n251), .o1(new_n280));
  nor043aa1n02x5               g185(.a(new_n280), .b(new_n278), .c(new_n269), .o1(new_n281));
  nano22aa1n03x7               g186(.a(new_n281), .b(new_n273), .c(new_n274), .out0(new_n282));
  norp02aa1n02x5               g187(.a(new_n277), .b(new_n282), .o1(\s[28] ));
  and002aa1n02x5               g188(.a(new_n274), .b(new_n271), .o(new_n284));
  oai012aa1n03x5               g189(.a(new_n284), .b(new_n280), .c(new_n278), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n286));
  xnrc02aa1n02x5               g191(.a(\b[28] ), .b(\a[29] ), .out0(new_n287));
  aoi012aa1n03x5               g192(.a(new_n287), .b(new_n285), .c(new_n286), .o1(new_n288));
  aobi12aa1n03x5               g193(.a(new_n284), .b(new_n268), .c(new_n264), .out0(new_n289));
  nano22aa1n03x5               g194(.a(new_n289), .b(new_n286), .c(new_n287), .out0(new_n290));
  norp02aa1n03x5               g195(.a(new_n288), .b(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g197(.a(new_n287), .b(new_n274), .c(new_n271), .out0(new_n293));
  oai012aa1n03x5               g198(.a(new_n293), .b(new_n280), .c(new_n278), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n286), .carry(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .out0(new_n296));
  aoi012aa1n03x5               g201(.a(new_n296), .b(new_n294), .c(new_n295), .o1(new_n297));
  aobi12aa1n03x5               g202(.a(new_n293), .b(new_n268), .c(new_n264), .out0(new_n298));
  nano22aa1n03x5               g203(.a(new_n298), .b(new_n295), .c(new_n296), .out0(new_n299));
  norp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[30] ));
  nano23aa1n02x4               g205(.a(new_n296), .b(new_n287), .c(new_n274), .d(new_n271), .out0(new_n301));
  aobi12aa1n02x7               g206(.a(new_n301), .b(new_n268), .c(new_n264), .out0(new_n302));
  oao003aa1n02x5               g207(.a(\a[30] ), .b(\b[29] ), .c(new_n295), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  nano22aa1n03x5               g209(.a(new_n302), .b(new_n303), .c(new_n304), .out0(new_n305));
  oaih12aa1n02x5               g210(.a(new_n301), .b(new_n280), .c(new_n278), .o1(new_n306));
  aoi012aa1n03x5               g211(.a(new_n304), .b(new_n306), .c(new_n303), .o1(new_n307));
  norp02aa1n03x5               g212(.a(new_n307), .b(new_n305), .o1(\s[31] ));
  xnrb03aa1n02x5               g213(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g214(.a(\a[3] ), .b(\b[2] ), .c(new_n114), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g217(.a(new_n100), .b(new_n102), .c(new_n121), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(new_n101), .out0(\s[6] ));
  oaoi03aa1n02x5               g219(.a(\a[6] ), .b(\b[5] ), .c(new_n313), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oai012aa1n02x5               g221(.a(new_n107), .b(new_n315), .c(new_n106), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi112aa1n02x5               g223(.a(new_n110), .b(new_n125), .c(new_n121), .d(new_n124), .o1(new_n319));
  norb02aa1n02x5               g224(.a(new_n126), .b(new_n319), .out0(\s[9] ));
endmodule


