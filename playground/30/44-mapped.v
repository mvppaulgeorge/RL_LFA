// Benchmark "adder" written by ABC on Thu Jul 18 03:44:29 2024

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
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n314, new_n316, new_n318, new_n320,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n24x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  oa0022aa1n09x5               g002(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n98));
  tech160nm_fixnrc02aa1n04x5   g003(.a(\b[2] ), .b(\a[3] ), .out0(new_n99));
  nand02aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n09x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nor042aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  oai012aa1n12x5               g007(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n103));
  oai012aa1n12x5               g008(.a(new_n98), .b(new_n99), .c(new_n103), .o1(new_n104));
  nor002aa1n03x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nanp02aa1n09x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  norb02aa1n06x4               g011(.a(new_n106), .b(new_n105), .out0(new_n107));
  xnrc02aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .out0(new_n108));
  inv000aa1d42x5               g013(.a(\a[7] ), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\b[6] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(new_n110), .b(new_n109), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nor022aa1n16x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanp02aa1n04x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nanb03aa1n06x5               g020(.a(new_n114), .b(new_n115), .c(new_n113), .out0(new_n116));
  nano23aa1n03x7               g021(.a(new_n116), .b(new_n108), .c(new_n111), .d(new_n112), .out0(new_n117));
  nand23aa1n09x5               g022(.a(new_n117), .b(new_n104), .c(new_n107), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[7] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\a[8] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n119), .b(new_n120), .o1(new_n121));
  aoai13aa1n06x5               g026(.a(new_n112), .b(new_n105), .c(new_n114), .d(new_n106), .o1(new_n122));
  nanp03aa1n02x5               g027(.a(new_n122), .b(new_n121), .c(new_n111), .o1(new_n123));
  oaib12aa1n06x5               g028(.a(new_n123), .b(new_n119), .c(\a[8] ), .out0(new_n124));
  tech160nm_fixnrc02aa1n04x5   g029(.a(\b[8] ), .b(\a[9] ), .out0(new_n125));
  aoai13aa1n06x5               g030(.a(new_n97), .b(new_n125), .c(new_n118), .d(new_n124), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  xnrc02aa1n12x5               g032(.a(\b[9] ), .b(\a[10] ), .out0(new_n128));
  inv000aa1d42x5               g033(.a(new_n128), .o1(new_n129));
  oaoi03aa1n12x5               g034(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n130));
  nor002aa1d32x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand22aa1n09x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  aoai13aa1n06x5               g038(.a(new_n133), .b(new_n130), .c(new_n126), .d(new_n129), .o1(new_n134));
  aoi112aa1n02x5               g039(.a(new_n133), .b(new_n130), .c(new_n126), .d(new_n129), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(\s[11] ));
  inv040aa1n04x5               g041(.a(new_n131), .o1(new_n137));
  nor042aa1n06x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand22aa1n09x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n134), .c(new_n137), .out0(\s[12] ));
  nano23aa1n09x5               g046(.a(new_n131), .b(new_n138), .c(new_n139), .d(new_n132), .out0(new_n142));
  nona22aa1n02x4               g047(.a(new_n142), .b(new_n128), .c(new_n125), .out0(new_n143));
  oaih22aa1n04x5               g048(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n144));
  aob012aa1n06x5               g049(.a(new_n144), .b(\b[9] ), .c(\a[10] ), .out0(new_n145));
  nona23aa1d18x5               g050(.a(new_n139), .b(new_n132), .c(new_n131), .d(new_n138), .out0(new_n146));
  oaoi03aa1n09x5               g051(.a(\a[12] ), .b(\b[11] ), .c(new_n137), .o1(new_n147));
  oabi12aa1n18x5               g052(.a(new_n147), .b(new_n146), .c(new_n145), .out0(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n143), .c(new_n118), .d(new_n124), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g056(.a(\a[14] ), .o1(new_n152));
  nor002aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  xnrc02aa1n06x5               g058(.a(\b[12] ), .b(\a[13] ), .out0(new_n154));
  aoib12aa1n02x5               g059(.a(new_n153), .b(new_n150), .c(new_n154), .out0(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(new_n152), .out0(\s[14] ));
  tech160nm_fixnrc02aa1n02p5x5 g061(.a(\b[13] ), .b(\a[14] ), .out0(new_n157));
  nor042aa1n04x5               g062(.a(new_n157), .b(new_n154), .o1(new_n158));
  inv000aa1d42x5               g063(.a(\b[13] ), .o1(new_n159));
  oao003aa1n03x5               g064(.a(new_n152), .b(new_n159), .c(new_n153), .carry(new_n160));
  aoi012aa1n02x5               g065(.a(new_n160), .b(new_n150), .c(new_n158), .o1(new_n161));
  nor042aa1n06x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  inv030aa1n02x5               g067(.a(new_n162), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  xnbna2aa1n03x5               g069(.a(new_n161), .b(new_n164), .c(new_n163), .out0(\s[15] ));
  norb02aa1n02x5               g070(.a(new_n164), .b(new_n162), .out0(new_n166));
  aoai13aa1n03x5               g071(.a(new_n166), .b(new_n160), .c(new_n150), .d(new_n158), .o1(new_n167));
  xnrc02aa1n12x5               g072(.a(\b[15] ), .b(\a[16] ), .out0(new_n168));
  inv000aa1d42x5               g073(.a(new_n168), .o1(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n167), .c(new_n163), .out0(\s[16] ));
  xorc02aa1n02x5               g075(.a(\a[8] ), .b(\b[7] ), .out0(new_n171));
  xorc02aa1n02x5               g076(.a(\a[7] ), .b(\b[6] ), .out0(new_n172));
  nano32aa1n02x4               g077(.a(new_n116), .b(new_n172), .c(new_n171), .d(new_n107), .out0(new_n173));
  and002aa1n02x5               g078(.a(\b[7] ), .b(\a[8] ), .o(new_n174));
  aoi013aa1n02x4               g079(.a(new_n174), .b(new_n122), .c(new_n111), .d(new_n121), .o1(new_n175));
  nano22aa1d15x5               g080(.a(new_n168), .b(new_n163), .c(new_n164), .out0(new_n176));
  nano22aa1n03x7               g081(.a(new_n143), .b(new_n158), .c(new_n176), .out0(new_n177));
  aoai13aa1n04x5               g082(.a(new_n177), .b(new_n175), .c(new_n173), .d(new_n104), .o1(new_n178));
  aoai13aa1n03x5               g083(.a(new_n176), .b(new_n160), .c(new_n148), .d(new_n158), .o1(new_n179));
  oao003aa1n12x5               g084(.a(\a[16] ), .b(\b[15] ), .c(new_n163), .carry(new_n180));
  nand23aa1n06x5               g085(.a(new_n178), .b(new_n179), .c(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g087(.a(\a[18] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n03x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  nor043aa1n03x5               g092(.a(new_n146), .b(new_n128), .c(new_n125), .o1(new_n188));
  nand23aa1n04x5               g093(.a(new_n188), .b(new_n158), .c(new_n176), .o1(new_n189));
  aoi012aa1d18x5               g094(.a(new_n189), .b(new_n118), .c(new_n124), .o1(new_n190));
  inv000aa1n02x5               g095(.a(new_n160), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n176), .o1(new_n192));
  aoai13aa1n06x5               g097(.a(new_n158), .b(new_n147), .c(new_n142), .d(new_n130), .o1(new_n193));
  aoai13aa1n12x5               g098(.a(new_n180), .b(new_n192), .c(new_n193), .d(new_n191), .o1(new_n194));
  xroi22aa1d04x5               g099(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n195));
  oai012aa1n06x5               g100(.a(new_n195), .b(new_n194), .c(new_n190), .o1(new_n196));
  oai022aa1d24x5               g101(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n197));
  oaib12aa1n18x5               g102(.a(new_n197), .b(new_n183), .c(\b[17] ), .out0(new_n198));
  nor002aa1d32x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nand02aa1n04x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  xobna2aa1n03x5               g106(.a(new_n201), .b(new_n196), .c(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g108(.a(new_n198), .o1(new_n204));
  oaoi13aa1n02x7               g109(.a(new_n204), .b(new_n195), .c(new_n194), .d(new_n190), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n199), .o1(new_n206));
  norp02aa1n09x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nand02aa1d06x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nanb02aa1n02x5               g113(.a(new_n207), .b(new_n208), .out0(new_n209));
  oaoi13aa1n02x7               g114(.a(new_n209), .b(new_n206), .c(new_n205), .d(new_n201), .o1(new_n210));
  tech160nm_fiaoi012aa1n03p5x5 g115(.a(new_n201), .b(new_n196), .c(new_n198), .o1(new_n211));
  nano22aa1n03x7               g116(.a(new_n211), .b(new_n206), .c(new_n209), .out0(new_n212));
  nor002aa1n02x5               g117(.a(new_n210), .b(new_n212), .o1(\s[20] ));
  nano23aa1n09x5               g118(.a(new_n199), .b(new_n207), .c(new_n208), .d(new_n200), .out0(new_n214));
  nand02aa1d04x5               g119(.a(new_n195), .b(new_n214), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  tech160nm_fioai012aa1n05x5   g121(.a(new_n216), .b(new_n194), .c(new_n190), .o1(new_n217));
  nona23aa1d18x5               g122(.a(new_n208), .b(new_n200), .c(new_n199), .d(new_n207), .out0(new_n218));
  aoi012aa1n12x5               g123(.a(new_n207), .b(new_n199), .c(new_n208), .o1(new_n219));
  oai012aa1d24x5               g124(.a(new_n219), .b(new_n218), .c(new_n198), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  nor042aa1n09x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  xnbna2aa1n03x5               g129(.a(new_n224), .b(new_n217), .c(new_n221), .out0(\s[21] ));
  inv000aa1d42x5               g130(.a(new_n222), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n224), .b(new_n220), .c(new_n181), .d(new_n216), .o1(new_n227));
  tech160nm_fixnrc02aa1n02p5x5 g132(.a(\b[21] ), .b(\a[22] ), .out0(new_n228));
  tech160nm_fiaoi012aa1n02p5x5 g133(.a(new_n228), .b(new_n227), .c(new_n226), .o1(new_n229));
  aobi12aa1n06x5               g134(.a(new_n224), .b(new_n217), .c(new_n221), .out0(new_n230));
  nano22aa1n03x5               g135(.a(new_n230), .b(new_n226), .c(new_n228), .out0(new_n231));
  norp02aa1n03x5               g136(.a(new_n229), .b(new_n231), .o1(\s[22] ));
  nano22aa1n03x7               g137(.a(new_n228), .b(new_n226), .c(new_n223), .out0(new_n233));
  and003aa1n02x5               g138(.a(new_n195), .b(new_n233), .c(new_n214), .o(new_n234));
  oai012aa1n06x5               g139(.a(new_n234), .b(new_n194), .c(new_n190), .o1(new_n235));
  oao003aa1n12x5               g140(.a(\a[22] ), .b(\b[21] ), .c(new_n226), .carry(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  tech160nm_fiaoi012aa1n03p5x5 g142(.a(new_n237), .b(new_n220), .c(new_n233), .o1(new_n238));
  xnrc02aa1n12x5               g143(.a(\b[22] ), .b(\a[23] ), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  xnbna2aa1n03x5               g145(.a(new_n240), .b(new_n235), .c(new_n238), .out0(\s[23] ));
  inv040aa1n03x5               g146(.a(new_n238), .o1(new_n242));
  oaoi13aa1n02x7               g147(.a(new_n242), .b(new_n234), .c(new_n194), .d(new_n190), .o1(new_n243));
  nor042aa1n03x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  tech160nm_fixnrc02aa1n02p5x5 g150(.a(\b[23] ), .b(\a[24] ), .out0(new_n246));
  oaoi13aa1n03x5               g151(.a(new_n246), .b(new_n245), .c(new_n243), .d(new_n239), .o1(new_n247));
  tech160nm_fiaoi012aa1n02p5x5 g152(.a(new_n239), .b(new_n235), .c(new_n238), .o1(new_n248));
  nano22aa1n02x4               g153(.a(new_n248), .b(new_n245), .c(new_n246), .out0(new_n249));
  norp02aa1n03x5               g154(.a(new_n247), .b(new_n249), .o1(\s[24] ));
  nor042aa1n09x5               g155(.a(new_n246), .b(new_n239), .o1(new_n251));
  nano22aa1n03x7               g156(.a(new_n215), .b(new_n233), .c(new_n251), .out0(new_n252));
  inv040aa1n02x5               g157(.a(new_n219), .o1(new_n253));
  aoai13aa1n06x5               g158(.a(new_n233), .b(new_n253), .c(new_n214), .d(new_n204), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n251), .o1(new_n255));
  oao003aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .c(new_n245), .carry(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n255), .c(new_n254), .d(new_n236), .o1(new_n257));
  oaoi13aa1n06x5               g162(.a(new_n257), .b(new_n252), .c(new_n194), .d(new_n190), .o1(new_n258));
  xnrb03aa1n03x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  tech160nm_fixnrc02aa1n04x5   g166(.a(\b[24] ), .b(\a[25] ), .out0(new_n262));
  xnrc02aa1n12x5               g167(.a(\b[25] ), .b(\a[26] ), .out0(new_n263));
  oaoi13aa1n03x5               g168(.a(new_n263), .b(new_n261), .c(new_n258), .d(new_n262), .o1(new_n264));
  oai012aa1n06x5               g169(.a(new_n252), .b(new_n194), .c(new_n190), .o1(new_n265));
  aoib12aa1n06x5               g170(.a(new_n262), .b(new_n265), .c(new_n257), .out0(new_n266));
  nano22aa1n03x7               g171(.a(new_n266), .b(new_n261), .c(new_n263), .out0(new_n267));
  norp02aa1n02x5               g172(.a(new_n264), .b(new_n267), .o1(\s[26] ));
  nor042aa1n04x5               g173(.a(new_n263), .b(new_n262), .o1(new_n269));
  nano32aa1n09x5               g174(.a(new_n215), .b(new_n269), .c(new_n233), .d(new_n251), .out0(new_n270));
  oai012aa1n12x5               g175(.a(new_n270), .b(new_n194), .c(new_n190), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .c(new_n261), .carry(new_n272));
  aobi12aa1n06x5               g177(.a(new_n272), .b(new_n257), .c(new_n269), .out0(new_n273));
  xorc02aa1n12x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n273), .c(new_n271), .out0(\s[27] ));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  inv040aa1n03x5               g181(.a(new_n276), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n251), .b(new_n237), .c(new_n220), .d(new_n233), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n269), .o1(new_n279));
  aoai13aa1n06x5               g184(.a(new_n272), .b(new_n279), .c(new_n278), .d(new_n256), .o1(new_n280));
  aoai13aa1n03x5               g185(.a(new_n274), .b(new_n280), .c(new_n181), .d(new_n270), .o1(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[27] ), .b(\a[28] ), .out0(new_n282));
  aoi012aa1n02x7               g187(.a(new_n282), .b(new_n281), .c(new_n277), .o1(new_n283));
  aobi12aa1n06x5               g188(.a(new_n274), .b(new_n273), .c(new_n271), .out0(new_n284));
  nano22aa1n03x5               g189(.a(new_n284), .b(new_n277), .c(new_n282), .out0(new_n285));
  nor002aa1n02x5               g190(.a(new_n283), .b(new_n285), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n274), .b(new_n282), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n280), .c(new_n181), .d(new_n270), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  aoi012aa1n03x5               g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  aobi12aa1n02x7               g196(.a(new_n287), .b(new_n273), .c(new_n271), .out0(new_n292));
  nano22aa1n03x5               g197(.a(new_n292), .b(new_n289), .c(new_n290), .out0(new_n293));
  norp02aa1n03x5               g198(.a(new_n291), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n274), .b(new_n290), .c(new_n282), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n296), .b(new_n280), .c(new_n181), .d(new_n270), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  tech160nm_fiaoi012aa1n02p5x5 g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n02x7               g205(.a(new_n296), .b(new_n273), .c(new_n271), .out0(new_n301));
  nano22aa1n03x5               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  norp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  xnrc02aa1n02x5               g208(.a(\b[30] ), .b(\a[31] ), .out0(new_n304));
  norb02aa1n02x5               g209(.a(new_n296), .b(new_n299), .out0(new_n305));
  aoai13aa1n02x5               g210(.a(new_n305), .b(new_n280), .c(new_n181), .d(new_n270), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n304), .b(new_n306), .c(new_n307), .o1(new_n308));
  aobi12aa1n03x5               g213(.a(new_n305), .b(new_n273), .c(new_n271), .out0(new_n309));
  nano22aa1n03x5               g214(.a(new_n309), .b(new_n304), .c(new_n307), .out0(new_n310));
  norp02aa1n03x5               g215(.a(new_n308), .b(new_n310), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  xnrc02aa1n02x5               g217(.a(\b[3] ), .b(\a[4] ), .out0(new_n313));
  oao003aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .carry(new_n314));
  mtn022aa1n02x5               g219(.a(new_n314), .b(new_n104), .sa(new_n313), .o1(\s[4] ));
  nanb02aa1n02x5               g220(.a(new_n114), .b(new_n115), .out0(new_n316));
  xnbna2aa1n03x5               g221(.a(new_n316), .b(new_n104), .c(new_n113), .out0(\s[5] ));
  aoi013aa1n02x4               g222(.a(new_n114), .b(new_n104), .c(new_n113), .d(new_n115), .o1(new_n318));
  xnrc02aa1n02x5               g223(.a(new_n318), .b(new_n107), .out0(\s[6] ));
  oaoi03aa1n02x5               g224(.a(\a[6] ), .b(\b[5] ), .c(new_n318), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g226(.a(new_n109), .b(new_n110), .c(new_n320), .o1(new_n322));
  xorb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(new_n120), .out0(\s[8] ));
  xobna2aa1n03x5               g228(.a(new_n125), .b(new_n118), .c(new_n124), .out0(\s[9] ));
endmodule


