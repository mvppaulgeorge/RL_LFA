// Benchmark "adder" written by ABC on Wed Jul 17 22:00:07 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n319, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d24x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n03x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nor022aa1n04x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  xorc02aa1n12x5               g005(.a(\a[9] ), .b(\b[8] ), .out0(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand22aa1n12x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor002aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  tech160nm_fioai012aa1n05x5   g009(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nand02aa1d04x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nor022aa1n16x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nona23aa1n03x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  tech160nm_fiaoi012aa1n03p5x5 g015(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n111));
  oai012aa1n06x5               g016(.a(new_n111), .b(new_n110), .c(new_n105), .o1(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[5] ), .b(\a[6] ), .out0(new_n113));
  nor042aa1n04x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand02aa1n08x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nor042aa1d18x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nona23aa1n02x4               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  xnrc02aa1n02x5               g023(.a(\b[7] ), .b(\a[8] ), .out0(new_n119));
  nor043aa1n03x5               g024(.a(new_n118), .b(new_n119), .c(new_n113), .o1(new_n120));
  nanp02aa1n03x5               g025(.a(new_n112), .b(new_n120), .o1(new_n121));
  inv000aa1n02x5               g026(.a(new_n115), .o1(new_n122));
  and002aa1n02x5               g027(.a(\b[7] ), .b(\a[8] ), .o(new_n123));
  aob012aa1n03x5               g028(.a(new_n116), .b(\b[5] ), .c(\a[6] ), .out0(new_n124));
  oai122aa1n06x5               g029(.a(new_n124), .b(\a[7] ), .c(\b[6] ), .d(\a[6] ), .e(\b[5] ), .o1(new_n125));
  nona22aa1n06x5               g030(.a(new_n125), .b(new_n123), .c(new_n122), .out0(new_n126));
  oai012aa1n09x5               g031(.a(new_n126), .b(\b[7] ), .c(\a[8] ), .o1(new_n127));
  nanb02aa1n09x5               g032(.a(new_n127), .b(new_n121), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n99), .b(new_n100), .c(new_n128), .d(new_n101), .o1(new_n129));
  inv000aa1d42x5               g034(.a(new_n98), .o1(new_n130));
  aoai13aa1n02x5               g035(.a(new_n101), .b(new_n127), .c(new_n112), .d(new_n120), .o1(new_n131));
  nona32aa1n02x4               g036(.a(new_n131), .b(new_n100), .c(new_n130), .d(new_n97), .out0(new_n132));
  nanp02aa1n02x5               g037(.a(new_n129), .b(new_n132), .o1(\s[10] ));
  tech160nm_finand02aa1n05x5   g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  orn002aa1n12x5               g039(.a(\a[11] ), .b(\b[10] ), .o(new_n135));
  nanp02aa1n02x5               g040(.a(new_n135), .b(new_n134), .o1(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n132), .c(new_n98), .out0(\s[11] ));
  nona22aa1n02x4               g042(.a(new_n132), .b(new_n136), .c(new_n130), .out0(new_n138));
  nor042aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand22aa1n04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n138), .c(new_n135), .out0(\s[12] ));
  nano22aa1n09x5               g047(.a(new_n139), .b(new_n134), .c(new_n140), .out0(new_n143));
  nano32aa1n03x5               g048(.a(new_n99), .b(new_n143), .c(new_n101), .d(new_n135), .out0(new_n144));
  aoai13aa1n06x5               g049(.a(new_n144), .b(new_n127), .c(new_n112), .d(new_n120), .o1(new_n145));
  tech160nm_fioai012aa1n03p5x5 g050(.a(new_n98), .b(\b[10] ), .c(\a[11] ), .o1(new_n146));
  oab012aa1n06x5               g051(.a(new_n146), .b(new_n97), .c(new_n100), .out0(new_n147));
  oaoi03aa1n09x5               g052(.a(\a[12] ), .b(\b[11] ), .c(new_n135), .o1(new_n148));
  aoi012aa1n02x5               g053(.a(new_n148), .b(new_n147), .c(new_n143), .o1(new_n149));
  nor042aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  nanp02aa1n04x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nanb02aa1n02x5               g056(.a(new_n150), .b(new_n151), .out0(new_n152));
  xobna2aa1n03x5               g057(.a(new_n152), .b(new_n145), .c(new_n149), .out0(\s[13] ));
  inv000aa1d42x5               g058(.a(\a[13] ), .o1(new_n154));
  inv000aa1d42x5               g059(.a(\b[12] ), .o1(new_n155));
  nand22aa1n03x5               g060(.a(new_n145), .b(new_n149), .o1(new_n156));
  oaoi03aa1n02x5               g061(.a(new_n154), .b(new_n155), .c(new_n156), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nand02aa1d08x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nano23aa1n09x5               g065(.a(new_n150), .b(new_n159), .c(new_n160), .d(new_n151), .out0(new_n161));
  aoai13aa1n12x5               g066(.a(new_n160), .b(new_n159), .c(new_n154), .d(new_n155), .o1(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  nor042aa1d18x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nand42aa1d28x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  aoai13aa1n03x5               g071(.a(new_n166), .b(new_n163), .c(new_n156), .d(new_n161), .o1(new_n167));
  aoi112aa1n02x5               g072(.a(new_n166), .b(new_n163), .c(new_n156), .d(new_n161), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(\s[15] ));
  inv000aa1d42x5               g074(.a(new_n164), .o1(new_n170));
  nor042aa1n06x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nand42aa1d28x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  aobi12aa1n06x5               g078(.a(new_n173), .b(new_n167), .c(new_n170), .out0(new_n174));
  nona22aa1n02x4               g079(.a(new_n167), .b(new_n173), .c(new_n164), .out0(new_n175));
  norb02aa1n03x4               g080(.a(new_n175), .b(new_n174), .out0(\s[16] ));
  nanb03aa1n06x5               g081(.a(new_n99), .b(new_n101), .c(new_n135), .out0(new_n177));
  nano23aa1d15x5               g082(.a(new_n164), .b(new_n171), .c(new_n172), .d(new_n165), .out0(new_n178));
  nano32aa1d15x5               g083(.a(new_n177), .b(new_n178), .c(new_n143), .d(new_n161), .out0(new_n179));
  aoai13aa1n06x5               g084(.a(new_n179), .b(new_n127), .c(new_n112), .d(new_n120), .o1(new_n180));
  aoai13aa1n04x5               g085(.a(new_n161), .b(new_n148), .c(new_n147), .d(new_n143), .o1(new_n181));
  nand42aa1n04x5               g086(.a(new_n181), .b(new_n162), .o1(new_n182));
  tech160nm_finand02aa1n05x5   g087(.a(new_n182), .b(new_n178), .o1(new_n183));
  tech160nm_fiaoi012aa1n03p5x5 g088(.a(new_n171), .b(new_n164), .c(new_n172), .o1(new_n184));
  nanp03aa1d12x5               g089(.a(new_n180), .b(new_n183), .c(new_n184), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g091(.a(\a[18] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\a[17] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[16] ), .o1(new_n189));
  oaoi03aa1n03x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  inv000aa1d42x5               g096(.a(new_n178), .o1(new_n192));
  aoai13aa1n12x5               g097(.a(new_n184), .b(new_n192), .c(new_n181), .d(new_n162), .o1(new_n193));
  xroi22aa1d06x4               g098(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n193), .c(new_n128), .d(new_n179), .o1(new_n195));
  oai022aa1d24x5               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  oaib12aa1n18x5               g101(.a(new_n196), .b(new_n187), .c(\b[17] ), .out0(new_n197));
  nor002aa1d32x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nand42aa1n16x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  xnbna2aa1n03x5               g106(.a(new_n201), .b(new_n195), .c(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g108(.a(new_n198), .o1(new_n204));
  inv040aa1n08x5               g109(.a(new_n197), .o1(new_n205));
  aoai13aa1n03x5               g110(.a(new_n201), .b(new_n205), .c(new_n185), .d(new_n194), .o1(new_n206));
  nor002aa1d24x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nand02aa1d28x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(new_n209));
  tech160nm_fiaoi012aa1n02p5x5 g114(.a(new_n209), .b(new_n206), .c(new_n204), .o1(new_n210));
  tech160nm_fiaoi012aa1n02p5x5 g115(.a(new_n200), .b(new_n195), .c(new_n197), .o1(new_n211));
  nano22aa1n03x5               g116(.a(new_n211), .b(new_n204), .c(new_n209), .out0(new_n212));
  nor002aa1n02x5               g117(.a(new_n210), .b(new_n212), .o1(\s[20] ));
  nano23aa1n09x5               g118(.a(new_n198), .b(new_n207), .c(new_n208), .d(new_n199), .out0(new_n214));
  nand02aa1d04x5               g119(.a(new_n194), .b(new_n214), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n193), .c(new_n128), .d(new_n179), .o1(new_n217));
  nona23aa1d18x5               g122(.a(new_n208), .b(new_n199), .c(new_n198), .d(new_n207), .out0(new_n218));
  aoi012aa1d18x5               g123(.a(new_n207), .b(new_n198), .c(new_n208), .o1(new_n219));
  oai012aa1d24x5               g124(.a(new_n219), .b(new_n218), .c(new_n197), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xnbna2aa1n03x5               g127(.a(new_n222), .b(new_n217), .c(new_n221), .out0(\s[21] ));
  orn002aa1n24x5               g128(.a(\a[21] ), .b(\b[20] ), .o(new_n224));
  aoai13aa1n06x5               g129(.a(new_n222), .b(new_n220), .c(new_n185), .d(new_n216), .o1(new_n225));
  xnrc02aa1n12x5               g130(.a(\b[21] ), .b(\a[22] ), .out0(new_n226));
  tech160nm_fiaoi012aa1n02p5x5 g131(.a(new_n226), .b(new_n225), .c(new_n224), .o1(new_n227));
  aobi12aa1n06x5               g132(.a(new_n222), .b(new_n217), .c(new_n221), .out0(new_n228));
  nano22aa1n02x4               g133(.a(new_n228), .b(new_n224), .c(new_n226), .out0(new_n229));
  nor002aa1n02x5               g134(.a(new_n227), .b(new_n229), .o1(\s[22] ));
  nanp02aa1n02x5               g135(.a(\b[20] ), .b(\a[21] ), .o1(new_n231));
  nano22aa1n12x5               g136(.a(new_n226), .b(new_n224), .c(new_n231), .out0(new_n232));
  and003aa1n02x5               g137(.a(new_n194), .b(new_n232), .c(new_n214), .o(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n193), .c(new_n128), .d(new_n179), .o1(new_n234));
  oaoi03aa1n09x5               g139(.a(\a[22] ), .b(\b[21] ), .c(new_n224), .o1(new_n235));
  aoi012aa1d18x5               g140(.a(new_n235), .b(new_n220), .c(new_n232), .o1(new_n236));
  xnrc02aa1n12x5               g141(.a(\b[22] ), .b(\a[23] ), .out0(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  xnbna2aa1n03x5               g143(.a(new_n238), .b(new_n234), .c(new_n236), .out0(\s[23] ));
  nor042aa1n06x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n236), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n238), .b(new_n242), .c(new_n185), .d(new_n233), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[23] ), .b(\a[24] ), .out0(new_n244));
  aoi012aa1n03x5               g149(.a(new_n244), .b(new_n243), .c(new_n241), .o1(new_n245));
  tech160nm_fiaoi012aa1n02p5x5 g150(.a(new_n237), .b(new_n234), .c(new_n236), .o1(new_n246));
  nano22aa1n03x5               g151(.a(new_n246), .b(new_n241), .c(new_n244), .out0(new_n247));
  nor002aa1n02x5               g152(.a(new_n245), .b(new_n247), .o1(\s[24] ));
  nor042aa1n02x5               g153(.a(new_n244), .b(new_n237), .o1(new_n249));
  nano22aa1n03x7               g154(.a(new_n215), .b(new_n232), .c(new_n249), .out0(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n193), .c(new_n128), .d(new_n179), .o1(new_n251));
  inv030aa1n02x5               g156(.a(new_n219), .o1(new_n252));
  aoai13aa1n09x5               g157(.a(new_n232), .b(new_n252), .c(new_n214), .d(new_n205), .o1(new_n253));
  inv000aa1n03x5               g158(.a(new_n235), .o1(new_n254));
  inv030aa1n02x5               g159(.a(new_n249), .o1(new_n255));
  oao003aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .c(new_n241), .carry(new_n256));
  aoai13aa1n12x5               g161(.a(new_n256), .b(new_n255), .c(new_n253), .d(new_n254), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  xnrc02aa1n12x5               g163(.a(\b[24] ), .b(\a[25] ), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  xnbna2aa1n03x5               g165(.a(new_n260), .b(new_n251), .c(new_n258), .out0(\s[25] ));
  nor042aa1n03x5               g166(.a(\b[24] ), .b(\a[25] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  aoai13aa1n06x5               g168(.a(new_n260), .b(new_n257), .c(new_n185), .d(new_n250), .o1(new_n264));
  xnrc02aa1n02x5               g169(.a(\b[25] ), .b(\a[26] ), .out0(new_n265));
  aoi012aa1n03x5               g170(.a(new_n265), .b(new_n264), .c(new_n263), .o1(new_n266));
  tech160nm_fiaoi012aa1n02p5x5 g171(.a(new_n259), .b(new_n251), .c(new_n258), .o1(new_n267));
  nano22aa1n03x5               g172(.a(new_n267), .b(new_n263), .c(new_n265), .out0(new_n268));
  nor002aa1n02x5               g173(.a(new_n266), .b(new_n268), .o1(\s[26] ));
  nor042aa1n06x5               g174(.a(new_n265), .b(new_n259), .o1(new_n270));
  nano32aa1n03x7               g175(.a(new_n215), .b(new_n270), .c(new_n232), .d(new_n249), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n193), .c(new_n128), .d(new_n179), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .c(new_n263), .carry(new_n273));
  aobi12aa1n12x5               g178(.a(new_n273), .b(new_n257), .c(new_n270), .out0(new_n274));
  xorc02aa1n12x5               g179(.a(\a[27] ), .b(\b[26] ), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n274), .c(new_n272), .out0(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  inv040aa1n03x5               g182(.a(new_n277), .o1(new_n278));
  aoai13aa1n04x5               g183(.a(new_n249), .b(new_n235), .c(new_n220), .d(new_n232), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n270), .o1(new_n280));
  aoai13aa1n06x5               g185(.a(new_n273), .b(new_n280), .c(new_n279), .d(new_n256), .o1(new_n281));
  aoai13aa1n02x5               g186(.a(new_n275), .b(new_n281), .c(new_n185), .d(new_n271), .o1(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  tech160nm_fiaoi012aa1n02p5x5 g188(.a(new_n283), .b(new_n282), .c(new_n278), .o1(new_n284));
  aobi12aa1n03x5               g189(.a(new_n275), .b(new_n274), .c(new_n272), .out0(new_n285));
  nano22aa1n03x5               g190(.a(new_n285), .b(new_n278), .c(new_n283), .out0(new_n286));
  norp02aa1n03x5               g191(.a(new_n284), .b(new_n286), .o1(\s[28] ));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  norb02aa1n02x5               g193(.a(new_n275), .b(new_n283), .out0(new_n289));
  aoai13aa1n02x5               g194(.a(new_n289), .b(new_n281), .c(new_n185), .d(new_n271), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n288), .b(new_n290), .c(new_n291), .o1(new_n292));
  aobi12aa1n03x5               g197(.a(new_n289), .b(new_n274), .c(new_n272), .out0(new_n293));
  nano22aa1n03x5               g198(.a(new_n293), .b(new_n288), .c(new_n291), .out0(new_n294));
  norp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  norb03aa1n02x5               g202(.a(new_n275), .b(new_n288), .c(new_n283), .out0(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n281), .c(new_n185), .d(new_n271), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n300));
  tech160nm_fiaoi012aa1n02p5x5 g205(.a(new_n297), .b(new_n299), .c(new_n300), .o1(new_n301));
  aobi12aa1n03x5               g206(.a(new_n298), .b(new_n274), .c(new_n272), .out0(new_n302));
  nano22aa1n03x5               g207(.a(new_n302), .b(new_n297), .c(new_n300), .out0(new_n303));
  norp02aa1n03x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  norb02aa1n02x5               g210(.a(new_n298), .b(new_n297), .out0(new_n306));
  aoai13aa1n02x5               g211(.a(new_n306), .b(new_n281), .c(new_n185), .d(new_n271), .o1(new_n307));
  oao003aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n300), .carry(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n305), .b(new_n307), .c(new_n308), .o1(new_n309));
  aobi12aa1n03x5               g214(.a(new_n306), .b(new_n274), .c(new_n272), .out0(new_n310));
  nano22aa1n03x5               g215(.a(new_n310), .b(new_n305), .c(new_n308), .out0(new_n311));
  norp02aa1n03x5               g216(.a(new_n309), .b(new_n311), .o1(\s[31] ));
  xnrb03aa1n02x5               g217(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g221(.a(new_n116), .b(new_n112), .c(new_n117), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g223(.a(\a[6] ), .b(\b[5] ), .c(new_n317), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g225(.a(new_n114), .b(new_n319), .c(new_n115), .o1(new_n321));
  xnrb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


