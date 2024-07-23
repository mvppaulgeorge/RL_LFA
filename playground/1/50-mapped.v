// Benchmark "adder" written by ABC on Wed Jul 17 12:53:51 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n317, new_n319, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixnrc02aa1n05x5   g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  nor002aa1d32x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(new_n98), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1d04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nor002aa1n03x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  tech160nm_fioai012aa1n05x5   g007(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n103));
  nor022aa1n06x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand02aa1n03x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n09x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  oai012aa1n02x7               g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  oai012aa1n12x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[5] ), .b(\a[6] ), .out0(new_n111));
  norp02aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand02aa1d04x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor042aa1n04x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n09x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  nor043aa1n03x5               g022(.a(new_n116), .b(new_n117), .c(new_n111), .o1(new_n118));
  oai012aa1n02x5               g023(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[5] ), .o1(new_n120));
  oai022aa1n02x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  oaib12aa1n02x5               g026(.a(new_n121), .b(new_n120), .c(\a[6] ), .out0(new_n122));
  tech160nm_fioai012aa1n04x5   g027(.a(new_n119), .b(new_n116), .c(new_n122), .o1(new_n123));
  nand02aa1n06x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nanb02aa1d24x5               g029(.a(new_n98), .b(new_n124), .out0(new_n125));
  inv000aa1d42x5               g030(.a(new_n125), .o1(new_n126));
  aoai13aa1n06x5               g031(.a(new_n126), .b(new_n123), .c(new_n110), .d(new_n118), .o1(new_n127));
  xobna2aa1n03x5               g032(.a(new_n97), .b(new_n127), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g033(.a(\a[10] ), .o1(new_n129));
  inv000aa1d42x5               g034(.a(\b[9] ), .o1(new_n130));
  nona22aa1n02x4               g035(.a(new_n127), .b(new_n98), .c(new_n97), .out0(new_n131));
  nor002aa1d32x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand02aa1d06x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  oai112aa1n03x5               g039(.a(new_n131), .b(new_n134), .c(new_n130), .d(new_n129), .o1(new_n135));
  oaoi13aa1n02x5               g040(.a(new_n134), .b(new_n131), .c(new_n129), .d(new_n130), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(\s[11] ));
  inv040aa1n06x5               g042(.a(new_n132), .o1(new_n138));
  nor022aa1n16x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand02aa1n06x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n135), .c(new_n138), .out0(\s[12] ));
  nona23aa1n09x5               g047(.a(new_n140), .b(new_n133), .c(new_n132), .d(new_n139), .out0(new_n143));
  tech160nm_fioaoi03aa1n03p5x5 g048(.a(new_n129), .b(new_n130), .c(new_n98), .o1(new_n144));
  oaoi03aa1n09x5               g049(.a(\a[12] ), .b(\b[11] ), .c(new_n138), .o1(new_n145));
  oabi12aa1n06x5               g050(.a(new_n145), .b(new_n143), .c(new_n144), .out0(new_n146));
  norp02aa1n02x5               g051(.a(new_n143), .b(new_n97), .o1(new_n147));
  inv000aa1n02x5               g052(.a(new_n147), .o1(new_n148));
  oab012aa1n06x5               g053(.a(new_n146), .b(new_n127), .c(new_n148), .out0(new_n149));
  nor002aa1n20x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  nand02aa1d28x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  xnbna2aa1n03x5               g057(.a(new_n149), .b(new_n152), .c(new_n151), .out0(\s[13] ));
  oaoi03aa1n02x5               g058(.a(\a[13] ), .b(\b[12] ), .c(new_n149), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  tech160nm_fiao0012aa1n03p5x5 g060(.a(new_n123), .b(new_n110), .c(new_n118), .o(new_n156));
  nor042aa1n04x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nand42aa1n08x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nano23aa1d15x5               g063(.a(new_n150), .b(new_n157), .c(new_n158), .d(new_n152), .out0(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  nona32aa1n03x5               g065(.a(new_n156), .b(new_n160), .c(new_n148), .d(new_n125), .out0(new_n161));
  aoi012aa1n02x5               g066(.a(new_n157), .b(new_n150), .c(new_n158), .o1(new_n162));
  aobi12aa1n06x5               g067(.a(new_n162), .b(new_n146), .c(new_n159), .out0(new_n163));
  nor042aa1n12x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nand02aa1d08x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  xnbna2aa1n03x5               g071(.a(new_n166), .b(new_n161), .c(new_n163), .out0(\s[15] ));
  inv000aa1d42x5               g072(.a(new_n164), .o1(new_n168));
  inv000aa1n02x5               g073(.a(new_n166), .o1(new_n169));
  aoai13aa1n03x5               g074(.a(new_n168), .b(new_n169), .c(new_n161), .d(new_n163), .o1(new_n170));
  xorb03aa1n02x5               g075(.a(new_n170), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  nor042aa1n04x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nand42aa1n04x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nano23aa1d15x5               g078(.a(new_n164), .b(new_n172), .c(new_n173), .d(new_n165), .out0(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  nano23aa1n09x5               g080(.a(new_n132), .b(new_n139), .c(new_n140), .d(new_n133), .out0(new_n176));
  nona22aa1n09x5               g081(.a(new_n176), .b(new_n97), .c(new_n125), .out0(new_n177));
  nano22aa1d15x5               g082(.a(new_n177), .b(new_n159), .c(new_n174), .out0(new_n178));
  aoai13aa1n06x5               g083(.a(new_n178), .b(new_n123), .c(new_n110), .d(new_n118), .o1(new_n179));
  aoi012aa1n02x7               g084(.a(new_n172), .b(new_n164), .c(new_n173), .o1(new_n180));
  oai112aa1n06x5               g085(.a(new_n179), .b(new_n180), .c(new_n163), .d(new_n175), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g087(.a(\a[18] ), .o1(new_n183));
  inv040aa1d32x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n03x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  tech160nm_fioaoi03aa1n03p5x5 g092(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n188));
  aoai13aa1n06x5               g093(.a(new_n159), .b(new_n145), .c(new_n176), .d(new_n188), .o1(new_n189));
  aoai13aa1n04x5               g094(.a(new_n180), .b(new_n175), .c(new_n189), .d(new_n162), .o1(new_n190));
  xroi22aa1d06x4               g095(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n190), .c(new_n156), .d(new_n178), .o1(new_n192));
  oai022aa1n09x5               g097(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n193));
  oaib12aa1n18x5               g098(.a(new_n193), .b(new_n183), .c(\b[17] ), .out0(new_n194));
  nor002aa1d32x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nand02aa1d16x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n192), .c(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g105(.a(new_n195), .o1(new_n201));
  aoi012aa1n03x5               g106(.a(new_n197), .b(new_n192), .c(new_n194), .o1(new_n202));
  nor002aa1n16x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nand02aa1d28x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  nano22aa1n03x5               g110(.a(new_n202), .b(new_n201), .c(new_n205), .out0(new_n206));
  nanp02aa1n02x5               g111(.a(new_n185), .b(new_n184), .o1(new_n207));
  oaoi03aa1n06x5               g112(.a(\a[18] ), .b(\b[17] ), .c(new_n207), .o1(new_n208));
  aoai13aa1n03x5               g113(.a(new_n198), .b(new_n208), .c(new_n181), .d(new_n191), .o1(new_n209));
  aoi012aa1n02x7               g114(.a(new_n205), .b(new_n209), .c(new_n201), .o1(new_n210));
  nor002aa1n02x5               g115(.a(new_n210), .b(new_n206), .o1(\s[20] ));
  nano23aa1n09x5               g116(.a(new_n195), .b(new_n203), .c(new_n204), .d(new_n196), .out0(new_n212));
  nand02aa1d04x5               g117(.a(new_n191), .b(new_n212), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n06x5               g119(.a(new_n214), .b(new_n190), .c(new_n156), .d(new_n178), .o1(new_n215));
  nona23aa1d18x5               g120(.a(new_n204), .b(new_n196), .c(new_n195), .d(new_n203), .out0(new_n216));
  aoi012aa1d18x5               g121(.a(new_n203), .b(new_n195), .c(new_n204), .o1(new_n217));
  oai012aa1d24x5               g122(.a(new_n217), .b(new_n216), .c(new_n194), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  nor002aa1d32x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  nand42aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  norb02aa1n02x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  xnbna2aa1n03x5               g127(.a(new_n222), .b(new_n215), .c(new_n219), .out0(\s[21] ));
  inv000aa1d42x5               g128(.a(new_n220), .o1(new_n224));
  aobi12aa1n06x5               g129(.a(new_n222), .b(new_n215), .c(new_n219), .out0(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[21] ), .b(\a[22] ), .out0(new_n226));
  nano22aa1n03x7               g131(.a(new_n225), .b(new_n224), .c(new_n226), .out0(new_n227));
  aoai13aa1n03x5               g132(.a(new_n222), .b(new_n218), .c(new_n181), .d(new_n214), .o1(new_n228));
  aoi012aa1n03x5               g133(.a(new_n226), .b(new_n228), .c(new_n224), .o1(new_n229));
  norp02aa1n03x5               g134(.a(new_n229), .b(new_n227), .o1(\s[22] ));
  nano22aa1n12x5               g135(.a(new_n226), .b(new_n224), .c(new_n221), .out0(new_n231));
  and003aa1n02x5               g136(.a(new_n191), .b(new_n231), .c(new_n212), .o(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n190), .c(new_n156), .d(new_n178), .o1(new_n233));
  oao003aa1n12x5               g138(.a(\a[22] ), .b(\b[21] ), .c(new_n224), .carry(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoi012aa1d18x5               g140(.a(new_n235), .b(new_n218), .c(new_n231), .o1(new_n236));
  xnrc02aa1n12x5               g141(.a(\b[22] ), .b(\a[23] ), .out0(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  xnbna2aa1n03x5               g143(.a(new_n238), .b(new_n233), .c(new_n236), .out0(\s[23] ));
  nor042aa1n06x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aoi012aa1n02x5               g146(.a(new_n237), .b(new_n233), .c(new_n236), .o1(new_n242));
  tech160nm_fixnrc02aa1n02p5x5 g147(.a(\b[23] ), .b(\a[24] ), .out0(new_n243));
  nano22aa1n02x4               g148(.a(new_n242), .b(new_n241), .c(new_n243), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n236), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n238), .b(new_n245), .c(new_n181), .d(new_n232), .o1(new_n246));
  aoi012aa1n03x5               g151(.a(new_n243), .b(new_n246), .c(new_n241), .o1(new_n247));
  norp02aa1n03x5               g152(.a(new_n247), .b(new_n244), .o1(\s[24] ));
  nor002aa1n04x5               g153(.a(new_n243), .b(new_n237), .o1(new_n249));
  nano22aa1n06x5               g154(.a(new_n213), .b(new_n231), .c(new_n249), .out0(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n190), .c(new_n156), .d(new_n178), .o1(new_n251));
  inv020aa1n03x5               g156(.a(new_n217), .o1(new_n252));
  aoai13aa1n06x5               g157(.a(new_n231), .b(new_n252), .c(new_n212), .d(new_n208), .o1(new_n253));
  inv030aa1n02x5               g158(.a(new_n249), .o1(new_n254));
  oao003aa1n02x5               g159(.a(\a[24] ), .b(\b[23] ), .c(new_n241), .carry(new_n255));
  aoai13aa1n12x5               g160(.a(new_n255), .b(new_n254), .c(new_n253), .d(new_n234), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  xnrc02aa1n12x5               g162(.a(\b[24] ), .b(\a[25] ), .out0(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  xnbna2aa1n03x5               g164(.a(new_n259), .b(new_n251), .c(new_n257), .out0(\s[25] ));
  nor042aa1n03x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  tech160nm_fiaoi012aa1n05x5   g167(.a(new_n258), .b(new_n251), .c(new_n257), .o1(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[25] ), .b(\a[26] ), .out0(new_n264));
  nano22aa1n02x4               g169(.a(new_n263), .b(new_n262), .c(new_n264), .out0(new_n265));
  aoai13aa1n03x5               g170(.a(new_n259), .b(new_n256), .c(new_n181), .d(new_n250), .o1(new_n266));
  aoi012aa1n03x5               g171(.a(new_n264), .b(new_n266), .c(new_n262), .o1(new_n267));
  nor002aa1n02x5               g172(.a(new_n267), .b(new_n265), .o1(\s[26] ));
  nor042aa1n06x5               g173(.a(new_n264), .b(new_n258), .o1(new_n269));
  nano32aa1n03x7               g174(.a(new_n213), .b(new_n269), .c(new_n231), .d(new_n249), .out0(new_n270));
  aoai13aa1n06x5               g175(.a(new_n270), .b(new_n190), .c(new_n156), .d(new_n178), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .c(new_n262), .carry(new_n272));
  aobi12aa1n12x5               g177(.a(new_n272), .b(new_n256), .c(new_n269), .out0(new_n273));
  xorc02aa1n12x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n271), .c(new_n273), .out0(\s[27] ));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  inv040aa1n03x5               g181(.a(new_n276), .o1(new_n277));
  aobi12aa1n03x5               g182(.a(new_n274), .b(new_n271), .c(new_n273), .out0(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .out0(new_n279));
  nano22aa1n03x5               g184(.a(new_n278), .b(new_n277), .c(new_n279), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n249), .b(new_n235), .c(new_n218), .d(new_n231), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n269), .o1(new_n282));
  aoai13aa1n04x5               g187(.a(new_n272), .b(new_n282), .c(new_n281), .d(new_n255), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n274), .b(new_n283), .c(new_n181), .d(new_n270), .o1(new_n284));
  aoi012aa1n03x5               g189(.a(new_n279), .b(new_n284), .c(new_n277), .o1(new_n285));
  norp02aa1n03x5               g190(.a(new_n285), .b(new_n280), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n274), .b(new_n279), .out0(new_n287));
  aobi12aa1n03x5               g192(.a(new_n287), .b(new_n271), .c(new_n273), .out0(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  nano22aa1n03x5               g195(.a(new_n288), .b(new_n289), .c(new_n290), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n287), .b(new_n283), .c(new_n181), .d(new_n270), .o1(new_n292));
  aoi012aa1n03x5               g197(.a(new_n290), .b(new_n292), .c(new_n289), .o1(new_n293));
  norp02aa1n03x5               g198(.a(new_n293), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n274), .b(new_n290), .c(new_n279), .out0(new_n296));
  aobi12aa1n03x5               g201(.a(new_n296), .b(new_n271), .c(new_n273), .out0(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  nano22aa1n03x5               g204(.a(new_n297), .b(new_n298), .c(new_n299), .out0(new_n300));
  aoai13aa1n02x5               g205(.a(new_n296), .b(new_n283), .c(new_n181), .d(new_n270), .o1(new_n301));
  tech160nm_fiaoi012aa1n02p5x5 g206(.a(new_n299), .b(new_n301), .c(new_n298), .o1(new_n302));
  norp02aa1n03x5               g207(.a(new_n302), .b(new_n300), .o1(\s[30] ));
  norb02aa1n03x4               g208(.a(new_n296), .b(new_n299), .out0(new_n304));
  aobi12aa1n03x5               g209(.a(new_n304), .b(new_n271), .c(new_n273), .out0(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  nano22aa1n03x5               g212(.a(new_n305), .b(new_n306), .c(new_n307), .out0(new_n308));
  aoai13aa1n02x5               g213(.a(new_n304), .b(new_n283), .c(new_n181), .d(new_n270), .o1(new_n309));
  aoi012aa1n02x5               g214(.a(new_n307), .b(new_n309), .c(new_n306), .o1(new_n310));
  norp02aa1n03x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norp02aa1n02x5               g220(.a(\b[4] ), .b(\a[5] ), .o1(new_n316));
  aoib12aa1n02x5               g221(.a(new_n316), .b(new_n110), .c(new_n117), .out0(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(new_n120), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g223(.a(\a[6] ), .b(\b[5] ), .c(new_n317), .carry(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g225(.a(\a[7] ), .b(\b[6] ), .c(new_n319), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n156), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


