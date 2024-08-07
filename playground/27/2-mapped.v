// Benchmark "adder" written by ABC on Thu Jul 18 01:46:34 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n310,
    new_n313, new_n315, new_n316, new_n318;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n04x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  nor022aa1n04x5               g002(.a(\b[6] ), .b(\a[7] ), .o1(new_n98));
  nand02aa1d04x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  tech160nm_fiao0012aa1n02p5x5 g004(.a(new_n97), .b(new_n98), .c(new_n99), .o(new_n100));
  inv000aa1d42x5               g005(.a(\a[6] ), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\b[5] ), .o1(new_n102));
  nor042aa1n02x5               g007(.a(\b[4] ), .b(\a[5] ), .o1(new_n103));
  oaoi03aa1n02x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nona23aa1n09x5               g010(.a(new_n99), .b(new_n105), .c(new_n98), .d(new_n97), .out0(new_n106));
  oabi12aa1n06x5               g011(.a(new_n100), .b(new_n106), .c(new_n104), .out0(new_n107));
  inv040aa1d32x5               g012(.a(\a[4] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\b[3] ), .o1(new_n109));
  nand22aa1n03x5               g014(.a(new_n109), .b(new_n108), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nand02aa1n02x5               g016(.a(new_n110), .b(new_n111), .o1(new_n112));
  nor022aa1n04x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  oaoi03aa1n02x5               g018(.a(new_n108), .b(new_n109), .c(new_n113), .o1(new_n114));
  nand42aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nanb02aa1n06x5               g020(.a(new_n113), .b(new_n115), .out0(new_n116));
  nanp02aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  nand22aa1n04x5               g022(.a(\b[0] ), .b(\a[1] ), .o1(new_n118));
  nor042aa1n02x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  oai012aa1n06x5               g024(.a(new_n117), .b(new_n119), .c(new_n118), .o1(new_n120));
  oai013aa1d12x5               g025(.a(new_n114), .b(new_n120), .c(new_n116), .d(new_n112), .o1(new_n121));
  tech160nm_fixnrc02aa1n02p5x5 g026(.a(\b[5] ), .b(\a[6] ), .out0(new_n122));
  xnrc02aa1n02x5               g027(.a(\b[4] ), .b(\a[5] ), .out0(new_n123));
  nor043aa1n06x5               g028(.a(new_n106), .b(new_n123), .c(new_n122), .o1(new_n124));
  aoi012aa1n02x5               g029(.a(new_n107), .b(new_n121), .c(new_n124), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  nor042aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  tech160nm_finand02aa1n05x5   g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  aoi012aa1n06x5               g035(.a(new_n129), .b(new_n128), .c(new_n130), .o1(new_n131));
  nand42aa1n03x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  nano23aa1n03x7               g037(.a(new_n128), .b(new_n129), .c(new_n130), .d(new_n132), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n107), .c(new_n121), .d(new_n124), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(new_n134), .b(new_n131), .o1(new_n135));
  xorb03aa1n02x5               g040(.a(new_n135), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv000aa1d42x5               g041(.a(\a[12] ), .o1(new_n137));
  nor022aa1n12x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nanp02aa1n04x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  aoi012aa1n02x5               g044(.a(new_n138), .b(new_n135), .c(new_n139), .o1(new_n140));
  xorb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(new_n137), .out0(\s[12] ));
  nor022aa1n12x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand02aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  tech160nm_fiaoi012aa1n05x5   g048(.a(new_n142), .b(new_n138), .c(new_n143), .o1(new_n144));
  nona23aa1d18x5               g049(.a(new_n143), .b(new_n139), .c(new_n138), .d(new_n142), .out0(new_n145));
  oai012aa1n18x5               g050(.a(new_n144), .b(new_n145), .c(new_n131), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n133), .b(new_n145), .out0(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n107), .c(new_n121), .d(new_n124), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n149), .b(new_n147), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g056(.a(\a[14] ), .o1(new_n152));
  nor042aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand42aa1n08x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n153), .b(new_n150), .c(new_n154), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(new_n152), .out0(\s[14] ));
  nor042aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nand42aa1n06x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nona23aa1n02x4               g063(.a(new_n158), .b(new_n154), .c(new_n153), .d(new_n157), .out0(new_n159));
  aoi012aa1n03x5               g064(.a(new_n157), .b(new_n153), .c(new_n158), .o1(new_n160));
  nano23aa1n06x5               g065(.a(new_n153), .b(new_n157), .c(new_n158), .d(new_n154), .out0(new_n161));
  aobi12aa1n02x5               g066(.a(new_n160), .b(new_n146), .c(new_n161), .out0(new_n162));
  oai012aa1n02x5               g067(.a(new_n162), .b(new_n149), .c(new_n159), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nand42aa1n06x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nor002aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  tech160nm_finand02aa1n03p5x5 g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  aoai13aa1n02x5               g074(.a(new_n169), .b(new_n165), .c(new_n163), .d(new_n166), .o1(new_n170));
  aoi112aa1n02x5               g075(.a(new_n165), .b(new_n169), .c(new_n163), .d(new_n166), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(\s[16] ));
  nanb02aa1n02x5               g077(.a(new_n165), .b(new_n166), .out0(new_n173));
  aoi012aa1n02x5               g078(.a(new_n167), .b(new_n165), .c(new_n168), .o1(new_n174));
  oai013aa1n02x5               g079(.a(new_n174), .b(new_n160), .c(new_n173), .d(new_n169), .o1(new_n175));
  nano23aa1n03x5               g080(.a(new_n165), .b(new_n167), .c(new_n168), .d(new_n166), .out0(new_n176));
  nanp02aa1n03x5               g081(.a(new_n176), .b(new_n161), .o1(new_n177));
  aoib12aa1n12x5               g082(.a(new_n175), .b(new_n146), .c(new_n177), .out0(new_n178));
  nano23aa1n06x5               g083(.a(new_n138), .b(new_n142), .c(new_n143), .d(new_n139), .out0(new_n179));
  nano22aa1n03x7               g084(.a(new_n177), .b(new_n133), .c(new_n179), .out0(new_n180));
  aoai13aa1n12x5               g085(.a(new_n180), .b(new_n107), .c(new_n121), .d(new_n124), .o1(new_n181));
  nor042aa1d18x5               g086(.a(\b[16] ), .b(\a[17] ), .o1(new_n182));
  nand42aa1n02x5               g087(.a(\b[16] ), .b(\a[17] ), .o1(new_n183));
  norb02aa1n02x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  xnbna2aa1n03x5               g089(.a(new_n184), .b(new_n181), .c(new_n178), .out0(\s[17] ));
  nanp02aa1n06x5               g090(.a(new_n181), .b(new_n178), .o1(new_n186));
  tech160nm_fiaoi012aa1n05x5   g091(.a(new_n182), .b(new_n186), .c(new_n184), .o1(new_n187));
  inv040aa1d32x5               g092(.a(\a[18] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[17] ), .o1(new_n189));
  nand22aa1n12x5               g094(.a(new_n189), .b(new_n188), .o1(new_n190));
  nand22aa1n09x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  xnbna2aa1n03x5               g096(.a(new_n187), .b(new_n191), .c(new_n190), .out0(\s[18] ));
  aob012aa1d18x5               g097(.a(new_n190), .b(new_n182), .c(new_n191), .out0(new_n193));
  inv000aa1d42x5               g098(.a(new_n193), .o1(new_n194));
  nano32aa1d12x5               g099(.a(new_n182), .b(new_n191), .c(new_n183), .d(new_n190), .out0(new_n195));
  inv000aa1d42x5               g100(.a(new_n195), .o1(new_n196));
  aoai13aa1n06x5               g101(.a(new_n194), .b(new_n196), .c(new_n181), .d(new_n178), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  norb02aa1n09x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  nor042aa1n09x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nand22aa1n09x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  norb02aa1d21x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  aoai13aa1n03x5               g111(.a(new_n206), .b(new_n200), .c(new_n197), .d(new_n202), .o1(new_n207));
  nand42aa1n03x5               g112(.a(new_n197), .b(new_n202), .o1(new_n208));
  nona22aa1n02x4               g113(.a(new_n208), .b(new_n206), .c(new_n200), .out0(new_n209));
  nanp02aa1n02x5               g114(.a(new_n209), .b(new_n207), .o1(\s[20] ));
  aoi012aa1d18x5               g115(.a(new_n203), .b(new_n200), .c(new_n204), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  nano23aa1n06x5               g117(.a(new_n200), .b(new_n203), .c(new_n204), .d(new_n201), .out0(new_n213));
  aoi012aa1n02x5               g118(.a(new_n212), .b(new_n213), .c(new_n193), .o1(new_n214));
  nand02aa1d04x5               g119(.a(new_n195), .b(new_n213), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n214), .b(new_n215), .c(new_n181), .d(new_n178), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g122(.a(\b[20] ), .b(\a[21] ), .o1(new_n218));
  nand02aa1d08x5               g123(.a(\b[20] ), .b(\a[21] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  nor002aa1n04x5               g125(.a(\b[21] ), .b(\a[22] ), .o1(new_n221));
  nand02aa1d08x5               g126(.a(\b[21] ), .b(\a[22] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n218), .c(new_n216), .d(new_n220), .o1(new_n225));
  nand42aa1n03x5               g130(.a(new_n216), .b(new_n220), .o1(new_n226));
  nona22aa1n02x4               g131(.a(new_n226), .b(new_n224), .c(new_n218), .out0(new_n227));
  nanp02aa1n03x5               g132(.a(new_n227), .b(new_n225), .o1(\s[22] ));
  nand23aa1d12x5               g133(.a(new_n193), .b(new_n202), .c(new_n205), .o1(new_n229));
  nanp02aa1n02x5               g134(.a(new_n229), .b(new_n211), .o1(new_n230));
  ao0012aa1n03x5               g135(.a(new_n221), .b(new_n218), .c(new_n222), .o(new_n231));
  nano23aa1n06x5               g136(.a(new_n218), .b(new_n221), .c(new_n222), .d(new_n219), .out0(new_n232));
  aoi012aa1n02x5               g137(.a(new_n231), .b(new_n230), .c(new_n232), .o1(new_n233));
  nanp03aa1n02x5               g138(.a(new_n195), .b(new_n213), .c(new_n232), .o1(new_n234));
  aoai13aa1n06x5               g139(.a(new_n233), .b(new_n234), .c(new_n181), .d(new_n178), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n04x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  nanp02aa1n04x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(new_n239));
  nor042aa1n04x5               g144(.a(\b[23] ), .b(\a[24] ), .o1(new_n240));
  nand02aa1d04x5               g145(.a(\b[23] ), .b(\a[24] ), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n241), .b(new_n240), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n243), .b(new_n237), .c(new_n235), .d(new_n239), .o1(new_n244));
  nand42aa1n03x5               g149(.a(new_n235), .b(new_n239), .o1(new_n245));
  nona22aa1n02x4               g150(.a(new_n245), .b(new_n243), .c(new_n237), .out0(new_n246));
  nanp02aa1n03x5               g151(.a(new_n246), .b(new_n244), .o1(\s[24] ));
  tech160nm_fiao0012aa1n02p5x5 g152(.a(new_n240), .b(new_n237), .c(new_n241), .o(new_n248));
  nano23aa1n09x5               g153(.a(new_n237), .b(new_n240), .c(new_n241), .d(new_n238), .out0(new_n249));
  aoi012aa1n06x5               g154(.a(new_n248), .b(new_n249), .c(new_n231), .o1(new_n250));
  nand02aa1d04x5               g155(.a(new_n249), .b(new_n232), .o1(new_n251));
  aoai13aa1n12x5               g156(.a(new_n250), .b(new_n251), .c(new_n229), .d(new_n211), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  nanb03aa1n02x5               g158(.a(new_n251), .b(new_n195), .c(new_n213), .out0(new_n254));
  aoai13aa1n06x5               g159(.a(new_n253), .b(new_n254), .c(new_n181), .d(new_n178), .o1(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  tech160nm_fixorc02aa1n03p5x5 g162(.a(\a[25] ), .b(\b[24] ), .out0(new_n258));
  xnrc02aa1n02x5               g163(.a(\b[25] ), .b(\a[26] ), .out0(new_n259));
  aoai13aa1n03x5               g164(.a(new_n259), .b(new_n257), .c(new_n255), .d(new_n258), .o1(new_n260));
  nand42aa1n03x5               g165(.a(new_n255), .b(new_n258), .o1(new_n261));
  nona22aa1n02x4               g166(.a(new_n261), .b(new_n259), .c(new_n257), .out0(new_n262));
  nanp02aa1n03x5               g167(.a(new_n262), .b(new_n260), .o1(\s[26] ));
  inv000aa1d42x5               g168(.a(\a[26] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(\b[25] ), .o1(new_n265));
  oaoi03aa1n02x5               g170(.a(new_n264), .b(new_n265), .c(new_n257), .o1(new_n266));
  inv000aa1n02x5               g171(.a(new_n266), .o1(new_n267));
  norb02aa1n02x5               g172(.a(new_n258), .b(new_n259), .out0(new_n268));
  aoi012aa1n06x5               g173(.a(new_n267), .b(new_n252), .c(new_n268), .o1(new_n269));
  norb03aa1n03x5               g174(.a(new_n268), .b(new_n215), .c(new_n251), .out0(new_n270));
  inv020aa1n03x5               g175(.a(new_n270), .o1(new_n271));
  aoai13aa1n06x5               g176(.a(new_n269), .b(new_n271), .c(new_n181), .d(new_n178), .o1(new_n272));
  xorb03aa1n03x5               g177(.a(new_n272), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1n03x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  xorc02aa1n02x5               g179(.a(\a[27] ), .b(\b[26] ), .out0(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[27] ), .b(\a[28] ), .out0(new_n276));
  aoai13aa1n03x5               g181(.a(new_n276), .b(new_n274), .c(new_n272), .d(new_n275), .o1(new_n277));
  nand22aa1n03x5               g182(.a(new_n252), .b(new_n268), .o1(new_n278));
  nand02aa1d04x5               g183(.a(new_n278), .b(new_n266), .o1(new_n279));
  aoai13aa1n06x5               g184(.a(new_n275), .b(new_n279), .c(new_n186), .d(new_n270), .o1(new_n280));
  nona22aa1n03x5               g185(.a(new_n280), .b(new_n276), .c(new_n274), .out0(new_n281));
  nanp02aa1n03x5               g186(.a(new_n277), .b(new_n281), .o1(\s[28] ));
  inv000aa1d42x5               g187(.a(\a[28] ), .o1(new_n283));
  inv000aa1d42x5               g188(.a(\b[27] ), .o1(new_n284));
  oaoi03aa1n09x5               g189(.a(new_n283), .b(new_n284), .c(new_n274), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  norb02aa1n02x5               g191(.a(new_n275), .b(new_n276), .out0(new_n287));
  aoai13aa1n02x7               g192(.a(new_n287), .b(new_n279), .c(new_n186), .d(new_n270), .o1(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[28] ), .b(\a[29] ), .out0(new_n289));
  nona22aa1n03x5               g194(.a(new_n288), .b(new_n289), .c(new_n286), .out0(new_n290));
  aoai13aa1n03x5               g195(.a(new_n289), .b(new_n286), .c(new_n272), .d(new_n287), .o1(new_n291));
  nanp02aa1n03x5               g196(.a(new_n291), .b(new_n290), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n118), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  tech160nm_fioaoi03aa1n03p5x5 g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .o1(new_n294));
  norb03aa1n02x5               g199(.a(new_n275), .b(new_n289), .c(new_n276), .out0(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n294), .c(new_n272), .d(new_n295), .o1(new_n297));
  aoai13aa1n06x5               g202(.a(new_n295), .b(new_n279), .c(new_n186), .d(new_n270), .o1(new_n298));
  nona22aa1n03x5               g203(.a(new_n298), .b(new_n296), .c(new_n294), .out0(new_n299));
  nanp02aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[30] ));
  nanb02aa1n02x5               g205(.a(new_n296), .b(new_n294), .out0(new_n301));
  oai012aa1n02x5               g206(.a(new_n301), .b(\b[29] ), .c(\a[30] ), .o1(new_n302));
  norb02aa1n02x5               g207(.a(new_n295), .b(new_n296), .out0(new_n303));
  aoai13aa1n02x7               g208(.a(new_n303), .b(new_n279), .c(new_n186), .d(new_n270), .o1(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nona22aa1n03x5               g210(.a(new_n304), .b(new_n305), .c(new_n302), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n305), .b(new_n302), .c(new_n272), .d(new_n303), .o1(new_n307));
  nanp02aa1n03x5               g212(.a(new_n307), .b(new_n306), .o1(\s[31] ));
  xnrb03aa1n02x5               g213(.a(new_n120), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g214(.a(\a[3] ), .b(\b[2] ), .c(new_n120), .o1(new_n310));
  xorb03aa1n02x5               g215(.a(new_n310), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g216(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoib12aa1n02x5               g217(.a(new_n103), .b(new_n121), .c(new_n123), .out0(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(new_n101), .out0(\s[6] ));
  norp02aa1n02x5               g219(.a(new_n123), .b(new_n122), .o1(new_n315));
  aob012aa1n02x5               g220(.a(new_n104), .b(new_n121), .c(new_n315), .out0(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g222(.a(new_n98), .b(new_n316), .c(new_n105), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g224(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


