// Benchmark "adder" written by ABC on Wed Jul 17 17:19:02 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n160, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n313, new_n315, new_n317;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand02aa1n03x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nor002aa1n06x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor002aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  oai012aa1n02x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  oa0022aa1n02x5               g007(.a(\a[6] ), .b(\b[5] ), .c(\a[5] ), .d(\b[4] ), .o(new_n103));
  and002aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o(new_n104));
  nand02aa1d04x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nona23aa1n09x5               g010(.a(new_n99), .b(new_n105), .c(new_n101), .d(new_n100), .out0(new_n106));
  oai013aa1n03x5               g011(.a(new_n102), .b(new_n106), .c(new_n103), .d(new_n104), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nand22aa1n02x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  tech160nm_fioai012aa1n04x5   g015(.a(new_n108), .b(new_n110), .c(new_n109), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  norp02aa1n03x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nona23aa1n06x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  aoi012aa1n02x5               g021(.a(new_n112), .b(new_n114), .c(new_n113), .o1(new_n117));
  oaih12aa1n06x5               g022(.a(new_n117), .b(new_n116), .c(new_n111), .o1(new_n118));
  tech160nm_fixorc02aa1n02p5x5 g023(.a(\a[6] ), .b(\b[5] ), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[4] ), .b(\a[5] ), .out0(new_n120));
  norb03aa1n09x5               g025(.a(new_n119), .b(new_n106), .c(new_n120), .out0(new_n121));
  xorc02aa1n06x5               g026(.a(\a[9] ), .b(\b[8] ), .out0(new_n122));
  aoai13aa1n02x5               g027(.a(new_n122), .b(new_n107), .c(new_n121), .d(new_n118), .o1(new_n123));
  xorc02aa1n12x5               g028(.a(\a[10] ), .b(\b[9] ), .out0(new_n124));
  xnbna2aa1n03x5               g029(.a(new_n124), .b(new_n123), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g030(.a(new_n124), .o1(new_n126));
  aoi112aa1n03x5               g031(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n127));
  oab012aa1n04x5               g032(.a(new_n127), .b(\a[10] ), .c(\b[9] ), .out0(new_n128));
  aoai13aa1n03x5               g033(.a(new_n128), .b(new_n126), .c(new_n123), .d(new_n98), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nand42aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nor022aa1n06x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nor022aa1n08x5               g037(.a(\b[11] ), .b(\a[12] ), .o1(new_n133));
  nand02aa1n06x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  aoi112aa1n02x5               g040(.a(new_n135), .b(new_n132), .c(new_n129), .d(new_n131), .o1(new_n136));
  aoai13aa1n02x5               g041(.a(new_n135), .b(new_n132), .c(new_n129), .d(new_n131), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(\s[12] ));
  nano23aa1n06x5               g043(.a(new_n133), .b(new_n132), .c(new_n134), .d(new_n131), .out0(new_n139));
  nand23aa1d12x5               g044(.a(new_n139), .b(new_n122), .c(new_n124), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n140), .o1(new_n141));
  aoai13aa1n06x5               g046(.a(new_n141), .b(new_n107), .c(new_n121), .d(new_n118), .o1(new_n142));
  nona23aa1n09x5               g047(.a(new_n131), .b(new_n134), .c(new_n133), .d(new_n132), .out0(new_n143));
  oa0012aa1n03x5               g048(.a(new_n134), .b(new_n133), .c(new_n132), .o(new_n144));
  oabi12aa1n12x5               g049(.a(new_n144), .b(new_n128), .c(new_n143), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  xnrc02aa1n12x5               g051(.a(\b[12] ), .b(\a[13] ), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  xnbna2aa1n03x5               g053(.a(new_n148), .b(new_n142), .c(new_n146), .out0(\s[13] ));
  orn002aa1n24x5               g054(.a(\a[13] ), .b(\b[12] ), .o(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n147), .c(new_n142), .d(new_n146), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g057(.a(\b[13] ), .b(\a[14] ), .out0(new_n153));
  norp02aa1n02x5               g058(.a(new_n153), .b(new_n147), .o1(new_n154));
  inv000aa1n02x5               g059(.a(new_n154), .o1(new_n155));
  oaoi03aa1n12x5               g060(.a(\a[14] ), .b(\b[13] ), .c(new_n150), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  aoai13aa1n04x5               g062(.a(new_n157), .b(new_n155), .c(new_n142), .d(new_n146), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  tech160nm_finand02aa1n05x5   g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nor042aa1n02x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nor042aa1n02x5               g066(.a(\b[15] ), .b(\a[16] ), .o1(new_n162));
  nanp02aa1n04x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  aoi112aa1n02x5               g069(.a(new_n164), .b(new_n161), .c(new_n158), .d(new_n160), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n164), .b(new_n161), .c(new_n158), .d(new_n160), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(\s[16] ));
  nano23aa1n03x7               g072(.a(new_n162), .b(new_n161), .c(new_n163), .d(new_n160), .out0(new_n168));
  nona22aa1n12x5               g073(.a(new_n168), .b(new_n153), .c(new_n147), .out0(new_n169));
  nor042aa1n12x5               g074(.a(new_n169), .b(new_n140), .o1(new_n170));
  aoai13aa1n09x5               g075(.a(new_n170), .b(new_n107), .c(new_n118), .d(new_n121), .o1(new_n171));
  nanp02aa1n02x5               g076(.a(new_n161), .b(new_n163), .o1(new_n172));
  nand42aa1n04x5               g077(.a(new_n168), .b(new_n156), .o1(new_n173));
  oai112aa1n06x5               g078(.a(new_n173), .b(new_n172), .c(\b[15] ), .d(\a[16] ), .o1(new_n174));
  aoib12aa1n12x5               g079(.a(new_n174), .b(new_n145), .c(new_n169), .out0(new_n175));
  nand02aa1d10x5               g080(.a(new_n171), .b(new_n175), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g082(.a(\a[18] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\a[17] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[16] ), .o1(new_n180));
  oaoi03aa1n02x5               g085(.a(new_n179), .b(new_n180), .c(new_n176), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(new_n178), .out0(\s[18] ));
  xroi22aa1d04x5               g087(.a(new_n179), .b(\b[16] ), .c(new_n178), .d(\b[17] ), .out0(new_n183));
  nanp02aa1n02x5               g088(.a(\b[17] ), .b(\a[18] ), .o1(new_n184));
  nona22aa1n02x4               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(new_n185));
  oaib12aa1n02x5               g090(.a(new_n185), .b(\b[17] ), .c(new_n178), .out0(new_n186));
  nand42aa1n02x5               g091(.a(\b[18] ), .b(\a[19] ), .o1(new_n187));
  nor022aa1n04x5               g092(.a(\b[18] ), .b(\a[19] ), .o1(new_n188));
  norb02aa1n02x5               g093(.a(new_n187), .b(new_n188), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n186), .c(new_n176), .d(new_n183), .o1(new_n190));
  aoi112aa1n02x5               g095(.a(new_n189), .b(new_n186), .c(new_n176), .d(new_n183), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n190), .b(new_n191), .out0(\s[19] ));
  xnrc02aa1n02x5               g097(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n02x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  nanp02aa1n02x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  nona22aa1n02x5               g101(.a(new_n190), .b(new_n196), .c(new_n188), .out0(new_n197));
  orn002aa1n24x5               g102(.a(\a[19] ), .b(\b[18] ), .o(new_n198));
  aobi12aa1n06x5               g103(.a(new_n196), .b(new_n190), .c(new_n198), .out0(new_n199));
  norb02aa1n03x4               g104(.a(new_n197), .b(new_n199), .out0(\s[20] ));
  nano23aa1n03x7               g105(.a(new_n194), .b(new_n188), .c(new_n195), .d(new_n187), .out0(new_n201));
  nanp02aa1n02x5               g106(.a(new_n183), .b(new_n201), .o1(new_n202));
  norp02aa1n02x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  aoi013aa1n02x4               g108(.a(new_n203), .b(new_n184), .c(new_n179), .d(new_n180), .o1(new_n204));
  nona23aa1n06x5               g109(.a(new_n187), .b(new_n195), .c(new_n194), .d(new_n188), .out0(new_n205));
  oaoi03aa1n12x5               g110(.a(\a[20] ), .b(\b[19] ), .c(new_n198), .o1(new_n206));
  inv000aa1n02x5               g111(.a(new_n206), .o1(new_n207));
  oai012aa1n09x5               g112(.a(new_n207), .b(new_n205), .c(new_n204), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoai13aa1n04x5               g114(.a(new_n209), .b(new_n202), .c(new_n171), .d(new_n175), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  xorc02aa1n02x5               g118(.a(\a[22] ), .b(\b[21] ), .out0(new_n214));
  aoi112aa1n02x5               g119(.a(new_n212), .b(new_n214), .c(new_n210), .d(new_n213), .o1(new_n215));
  aoai13aa1n03x5               g120(.a(new_n214), .b(new_n212), .c(new_n210), .d(new_n213), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(\s[22] ));
  inv000aa1d42x5               g122(.a(\a[23] ), .o1(new_n218));
  inv000aa1d42x5               g123(.a(\a[21] ), .o1(new_n219));
  inv040aa1d32x5               g124(.a(\a[22] ), .o1(new_n220));
  xroi22aa1d06x4               g125(.a(new_n219), .b(\b[20] ), .c(new_n220), .d(\b[21] ), .out0(new_n221));
  inv000aa1n02x5               g126(.a(new_n221), .o1(new_n222));
  nano22aa1n02x4               g127(.a(new_n222), .b(new_n183), .c(new_n201), .out0(new_n223));
  aoai13aa1n06x5               g128(.a(new_n221), .b(new_n206), .c(new_n201), .d(new_n186), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[21] ), .o1(new_n225));
  oao003aa1n02x5               g130(.a(new_n220), .b(new_n225), .c(new_n212), .carry(new_n226));
  inv000aa1n02x5               g131(.a(new_n226), .o1(new_n227));
  nanp02aa1n02x5               g132(.a(new_n224), .b(new_n227), .o1(new_n228));
  tech160nm_fiaoi012aa1n05x5   g133(.a(new_n228), .b(new_n176), .c(new_n223), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[22] ), .c(new_n218), .out0(\s[23] ));
  and002aa1n02x5               g135(.a(\b[22] ), .b(\a[23] ), .o(new_n231));
  xorc02aa1n02x5               g136(.a(\a[24] ), .b(\b[23] ), .out0(new_n232));
  norp03aa1n02x5               g137(.a(new_n106), .b(new_n104), .c(new_n103), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n102), .b(new_n233), .out0(new_n234));
  nand42aa1n02x5               g139(.a(new_n121), .b(new_n118), .o1(new_n235));
  nanp02aa1n03x5               g140(.a(new_n235), .b(new_n234), .o1(new_n236));
  oabi12aa1n03x5               g141(.a(new_n174), .b(new_n146), .c(new_n169), .out0(new_n237));
  aoai13aa1n06x5               g142(.a(new_n223), .b(new_n237), .c(new_n236), .d(new_n170), .o1(new_n238));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  nona22aa1n02x4               g144(.a(new_n238), .b(new_n228), .c(new_n239), .out0(new_n240));
  aoib12aa1n02x5               g145(.a(new_n232), .b(new_n240), .c(new_n231), .out0(new_n241));
  aoi112aa1n03x5               g146(.a(new_n239), .b(new_n228), .c(new_n176), .d(new_n223), .o1(new_n242));
  norb03aa1n02x5               g147(.a(new_n232), .b(new_n242), .c(new_n231), .out0(new_n243));
  norp02aa1n02x5               g148(.a(new_n241), .b(new_n243), .o1(\s[24] ));
  inv040aa1d32x5               g149(.a(\a[24] ), .o1(new_n245));
  xroi22aa1d06x4               g150(.a(new_n218), .b(\b[22] ), .c(new_n245), .d(\b[23] ), .out0(new_n246));
  nanb03aa1n02x5               g151(.a(new_n202), .b(new_n246), .c(new_n221), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n246), .o1(new_n248));
  oai022aa1n02x5               g153(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n249));
  oaib12aa1n02x5               g154(.a(new_n249), .b(new_n245), .c(\b[23] ), .out0(new_n250));
  aoai13aa1n04x5               g155(.a(new_n250), .b(new_n248), .c(new_n224), .d(new_n227), .o1(new_n251));
  inv000aa1n02x5               g156(.a(new_n251), .o1(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n247), .c(new_n171), .d(new_n175), .o1(new_n253));
  xorb03aa1n02x5               g158(.a(new_n253), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g159(.a(\b[24] ), .b(\a[25] ), .o1(new_n255));
  xorc02aa1n02x5               g160(.a(\a[25] ), .b(\b[24] ), .out0(new_n256));
  norp02aa1n02x5               g161(.a(\b[25] ), .b(\a[26] ), .o1(new_n257));
  nand42aa1n03x5               g162(.a(\b[25] ), .b(\a[26] ), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n258), .b(new_n257), .out0(new_n259));
  aoi112aa1n02x5               g164(.a(new_n255), .b(new_n259), .c(new_n253), .d(new_n256), .o1(new_n260));
  aoai13aa1n03x5               g165(.a(new_n259), .b(new_n255), .c(new_n253), .d(new_n256), .o1(new_n261));
  norb02aa1n02x7               g166(.a(new_n261), .b(new_n260), .out0(\s[26] ));
  nanp02aa1n06x5               g167(.a(new_n256), .b(new_n259), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n263), .o1(new_n264));
  nano32aa1n03x7               g169(.a(new_n202), .b(new_n264), .c(new_n221), .d(new_n246), .out0(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n237), .c(new_n236), .d(new_n170), .o1(new_n266));
  oai012aa1n02x5               g171(.a(new_n258), .b(new_n257), .c(new_n255), .o1(new_n267));
  aobi12aa1n09x5               g172(.a(new_n267), .b(new_n251), .c(new_n264), .out0(new_n268));
  nor042aa1n03x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  norb02aa1n02x5               g175(.a(new_n270), .b(new_n269), .out0(new_n271));
  xnbna2aa1n06x5               g176(.a(new_n271), .b(new_n268), .c(new_n266), .out0(\s[27] ));
  xorc02aa1n02x5               g177(.a(\a[28] ), .b(\b[27] ), .out0(new_n273));
  inv000aa1d42x5               g178(.a(new_n269), .o1(new_n274));
  nanp03aa1n03x5               g179(.a(new_n268), .b(new_n266), .c(new_n274), .o1(new_n275));
  aoi012aa1n03x5               g180(.a(new_n273), .b(new_n275), .c(new_n270), .o1(new_n276));
  aobi12aa1n06x5               g181(.a(new_n265), .b(new_n171), .c(new_n175), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n246), .b(new_n226), .c(new_n208), .d(new_n221), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n267), .b(new_n263), .c(new_n278), .d(new_n250), .o1(new_n279));
  norp03aa1n02x5               g184(.a(new_n279), .b(new_n277), .c(new_n269), .o1(new_n280));
  nano22aa1n02x4               g185(.a(new_n280), .b(new_n270), .c(new_n273), .out0(new_n281));
  nor002aa1n02x5               g186(.a(new_n276), .b(new_n281), .o1(\s[28] ));
  and002aa1n02x5               g187(.a(new_n273), .b(new_n271), .o(new_n283));
  oai012aa1n02x5               g188(.a(new_n283), .b(new_n279), .c(new_n277), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n274), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n284), .c(new_n285), .o1(new_n287));
  aobi12aa1n06x5               g192(.a(new_n283), .b(new_n268), .c(new_n266), .out0(new_n288));
  nano22aa1n03x5               g193(.a(new_n288), .b(new_n285), .c(new_n286), .out0(new_n289));
  norp02aa1n03x5               g194(.a(new_n287), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g196(.a(new_n286), .b(new_n273), .c(new_n271), .out0(new_n292));
  oai012aa1n02x5               g197(.a(new_n292), .b(new_n279), .c(new_n277), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[29] ), .b(\a[30] ), .out0(new_n295));
  aoi012aa1n02x5               g200(.a(new_n295), .b(new_n293), .c(new_n294), .o1(new_n296));
  aobi12aa1n06x5               g201(.a(new_n292), .b(new_n268), .c(new_n266), .out0(new_n297));
  nano22aa1n03x5               g202(.a(new_n297), .b(new_n294), .c(new_n295), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[30] ));
  nano23aa1n02x4               g204(.a(new_n295), .b(new_n286), .c(new_n273), .d(new_n271), .out0(new_n300));
  aobi12aa1n06x5               g205(.a(new_n300), .b(new_n268), .c(new_n266), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nano22aa1n03x5               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n300), .b(new_n279), .c(new_n277), .o1(new_n305));
  aoi012aa1n03x5               g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n03x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xnrb03aa1n02x5               g212(.a(new_n111), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n118), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g216(.a(\b[4] ), .b(\a[5] ), .o1(new_n312));
  oai012aa1n02x5               g217(.a(new_n312), .b(new_n118), .c(new_n120), .o1(new_n313));
  xnrc02aa1n02x5               g218(.a(new_n313), .b(new_n119), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g219(.a(new_n104), .b(new_n313), .c(new_n119), .o(new_n315));
  xnrb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g221(.a(\a[7] ), .b(\b[6] ), .c(new_n315), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g223(.a(new_n122), .b(new_n235), .c(new_n234), .out0(\s[9] ));
endmodule


