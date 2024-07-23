// Benchmark "adder" written by ABC on Wed Jul 17 21:34:16 2024

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
    new_n133, new_n134, new_n135, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n312, new_n315, new_n316, new_n318, new_n319, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n03x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor022aa1n04x5               g002(.a(\b[3] ), .b(\a[4] ), .o1(new_n98));
  nand42aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nor022aa1n06x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  aoi012aa1n02x5               g005(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand02aa1n03x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  tech160nm_finor002aa1n03p5x5 g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  tech160nm_fioai012aa1n05x5   g009(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n03x5               g011(.a(new_n106), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n107));
  oai012aa1n04x7               g012(.a(new_n101), .b(new_n107), .c(new_n105), .o1(new_n108));
  xnrc02aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .out0(new_n109));
  xnrc02aa1n02x5               g014(.a(\b[4] ), .b(\a[5] ), .out0(new_n110));
  nor042aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n03x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  nor043aa1n03x5               g020(.a(new_n115), .b(new_n110), .c(new_n109), .o1(new_n116));
  inv000aa1d42x5               g021(.a(new_n113), .o1(new_n117));
  oaoi03aa1n02x5               g022(.a(\a[8] ), .b(\b[7] ), .c(new_n117), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[5] ), .o1(new_n119));
  oai022aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  oaib12aa1n02x5               g025(.a(new_n120), .b(new_n119), .c(\a[6] ), .out0(new_n121));
  oabi12aa1n02x5               g026(.a(new_n118), .b(new_n115), .c(new_n121), .out0(new_n122));
  inv030aa1n02x5               g027(.a(new_n122), .o1(new_n123));
  aob012aa1n09x5               g028(.a(new_n123), .b(new_n108), .c(new_n116), .out0(new_n124));
  nand42aa1n06x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  aoi012aa1n02x5               g030(.a(new_n97), .b(new_n124), .c(new_n125), .o1(new_n126));
  xnrb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand42aa1n06x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nor042aa1n03x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nano23aa1n06x5               g034(.a(new_n97), .b(new_n129), .c(new_n128), .d(new_n125), .out0(new_n130));
  oai022aa1n02x5               g035(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n131));
  aoi022aa1n06x5               g036(.a(new_n124), .b(new_n130), .c(new_n128), .d(new_n131), .o1(new_n132));
  nor042aa1n09x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nand42aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n132), .b(new_n135), .c(new_n134), .out0(\s[11] ));
  oaoi03aa1n03x5               g041(.a(\a[11] ), .b(\b[10] ), .c(new_n132), .o1(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n03x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanp02aa1n04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nano23aa1n09x5               g045(.a(new_n133), .b(new_n139), .c(new_n140), .d(new_n135), .out0(new_n141));
  nand02aa1d10x5               g046(.a(new_n141), .b(new_n130), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n142), .o1(new_n143));
  oai112aa1n02x5               g048(.a(new_n134), .b(new_n128), .c(new_n129), .d(new_n97), .o1(new_n144));
  nanb03aa1n03x5               g049(.a(new_n139), .b(new_n140), .c(new_n135), .out0(new_n145));
  tech160nm_fiaoi012aa1n05x5   g050(.a(new_n139), .b(new_n133), .c(new_n140), .o1(new_n146));
  tech160nm_fioai012aa1n03p5x5 g051(.a(new_n146), .b(new_n144), .c(new_n145), .o1(new_n147));
  nor042aa1n04x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  nand42aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n147), .c(new_n124), .d(new_n143), .o1(new_n151));
  aoi112aa1n02x5               g056(.a(new_n150), .b(new_n147), .c(new_n124), .d(new_n143), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n151), .b(new_n152), .out0(\s[13] ));
  inv000aa1n03x5               g058(.a(new_n148), .o1(new_n154));
  nor042aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nanb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(new_n157));
  xobna2aa1n03x5               g062(.a(new_n157), .b(new_n151), .c(new_n154), .out0(\s[14] ));
  nano23aa1n06x5               g063(.a(new_n148), .b(new_n155), .c(new_n156), .d(new_n149), .out0(new_n159));
  and003aa1n02x5               g064(.a(new_n130), .b(new_n159), .c(new_n141), .o(new_n160));
  oai012aa1n02x5               g065(.a(new_n128), .b(\b[10] ), .c(\a[11] ), .o1(new_n161));
  oab012aa1n04x5               g066(.a(new_n161), .b(new_n97), .c(new_n129), .out0(new_n162));
  nano22aa1n03x7               g067(.a(new_n139), .b(new_n135), .c(new_n140), .out0(new_n163));
  inv000aa1n02x5               g068(.a(new_n146), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n159), .b(new_n164), .c(new_n162), .d(new_n163), .o1(new_n165));
  oaoi03aa1n12x5               g070(.a(\a[14] ), .b(\b[13] ), .c(new_n154), .o1(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(new_n165), .b(new_n167), .o1(new_n168));
  nor042aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  tech160nm_finand02aa1n03p5x5 g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  aoai13aa1n02x5               g076(.a(new_n171), .b(new_n168), .c(new_n124), .d(new_n160), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(new_n171), .b(new_n168), .c(new_n124), .d(new_n160), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(\s[15] ));
  inv000aa1d42x5               g079(.a(new_n169), .o1(new_n175));
  nor042aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  tech160nm_finand02aa1n03p5x5 g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  xnbna2aa1n03x5               g083(.a(new_n178), .b(new_n172), .c(new_n175), .out0(\s[16] ));
  nano23aa1d18x5               g084(.a(new_n169), .b(new_n176), .c(new_n177), .d(new_n170), .out0(new_n180));
  nano22aa1d15x5               g085(.a(new_n142), .b(new_n159), .c(new_n180), .out0(new_n181));
  aoai13aa1n06x5               g086(.a(new_n181), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n182));
  aoai13aa1n02x7               g087(.a(new_n180), .b(new_n166), .c(new_n147), .d(new_n159), .o1(new_n183));
  aoi012aa1n02x5               g088(.a(new_n176), .b(new_n169), .c(new_n177), .o1(new_n184));
  nand23aa1n06x5               g089(.a(new_n182), .b(new_n183), .c(new_n184), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g091(.a(\a[18] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\a[17] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[16] ), .o1(new_n189));
  oaoi03aa1n03x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  inv000aa1d42x5               g096(.a(new_n180), .o1(new_n192));
  aoai13aa1n06x5               g097(.a(new_n184), .b(new_n192), .c(new_n165), .d(new_n167), .o1(new_n193));
  xroi22aa1d04x5               g098(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n193), .c(new_n124), .d(new_n181), .o1(new_n195));
  oai022aa1d24x5               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  oaib12aa1n18x5               g101(.a(new_n196), .b(new_n187), .c(\b[17] ), .out0(new_n197));
  xnrc02aa1n12x5               g102(.a(\b[18] ), .b(\a[19] ), .out0(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  xnbna2aa1n03x5               g104(.a(new_n199), .b(new_n195), .c(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n197), .o1(new_n204));
  aoai13aa1n03x5               g109(.a(new_n199), .b(new_n204), .c(new_n185), .d(new_n194), .o1(new_n205));
  xnrc02aa1n12x5               g110(.a(\b[19] ), .b(\a[20] ), .out0(new_n206));
  aoi012aa1n03x5               g111(.a(new_n206), .b(new_n205), .c(new_n203), .o1(new_n207));
  aoi012aa1n02x5               g112(.a(new_n198), .b(new_n195), .c(new_n197), .o1(new_n208));
  nano22aa1n02x4               g113(.a(new_n208), .b(new_n203), .c(new_n206), .out0(new_n209));
  norp02aa1n03x5               g114(.a(new_n207), .b(new_n209), .o1(\s[20] ));
  nor042aa1n02x5               g115(.a(new_n206), .b(new_n198), .o1(new_n211));
  and002aa1n02x5               g116(.a(new_n194), .b(new_n211), .o(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n193), .c(new_n124), .d(new_n181), .o1(new_n213));
  oao003aa1n09x5               g118(.a(\a[20] ), .b(\b[19] ), .c(new_n203), .carry(new_n214));
  oai013aa1d12x5               g119(.a(new_n214), .b(new_n198), .c(new_n206), .d(new_n197), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  xorc02aa1n12x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  xnbna2aa1n03x5               g122(.a(new_n217), .b(new_n213), .c(new_n216), .out0(\s[21] ));
  orn002aa1n24x5               g123(.a(\a[21] ), .b(\b[20] ), .o(new_n219));
  aoai13aa1n03x5               g124(.a(new_n217), .b(new_n215), .c(new_n185), .d(new_n212), .o1(new_n220));
  xnrc02aa1n12x5               g125(.a(\b[21] ), .b(\a[22] ), .out0(new_n221));
  aoi012aa1n03x5               g126(.a(new_n221), .b(new_n220), .c(new_n219), .o1(new_n222));
  aobi12aa1n02x5               g127(.a(new_n217), .b(new_n213), .c(new_n216), .out0(new_n223));
  nano22aa1n02x4               g128(.a(new_n223), .b(new_n219), .c(new_n221), .out0(new_n224));
  nor002aa1n02x5               g129(.a(new_n222), .b(new_n224), .o1(\s[22] ));
  norb02aa1n15x5               g130(.a(new_n217), .b(new_n221), .out0(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  nano22aa1n02x4               g132(.a(new_n227), .b(new_n194), .c(new_n211), .out0(new_n228));
  aoai13aa1n06x5               g133(.a(new_n228), .b(new_n193), .c(new_n124), .d(new_n181), .o1(new_n229));
  oaoi03aa1n09x5               g134(.a(\a[22] ), .b(\b[21] ), .c(new_n219), .o1(new_n230));
  aoi012aa1d24x5               g135(.a(new_n230), .b(new_n215), .c(new_n226), .o1(new_n231));
  xorc02aa1n12x5               g136(.a(\a[23] ), .b(\b[22] ), .out0(new_n232));
  xnbna2aa1n03x5               g137(.a(new_n232), .b(new_n229), .c(new_n231), .out0(\s[23] ));
  nor042aa1n06x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n231), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n232), .b(new_n236), .c(new_n185), .d(new_n228), .o1(new_n237));
  xorc02aa1n12x5               g142(.a(\a[24] ), .b(\b[23] ), .out0(new_n238));
  inv000aa1d42x5               g143(.a(new_n238), .o1(new_n239));
  aoi012aa1n03x5               g144(.a(new_n239), .b(new_n237), .c(new_n235), .o1(new_n240));
  aobi12aa1n06x5               g145(.a(new_n232), .b(new_n229), .c(new_n231), .out0(new_n241));
  nano22aa1n02x4               g146(.a(new_n241), .b(new_n235), .c(new_n239), .out0(new_n242));
  nor002aa1n02x5               g147(.a(new_n240), .b(new_n242), .o1(\s[24] ));
  inv000aa1d42x5               g148(.a(\a[23] ), .o1(new_n244));
  inv000aa1d42x5               g149(.a(\a[24] ), .o1(new_n245));
  xroi22aa1d04x5               g150(.a(new_n244), .b(\b[22] ), .c(new_n245), .d(\b[23] ), .out0(new_n246));
  nano32aa1n02x4               g151(.a(new_n227), .b(new_n246), .c(new_n194), .d(new_n211), .out0(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n193), .c(new_n124), .d(new_n181), .o1(new_n248));
  nona22aa1n09x5               g153(.a(new_n199), .b(new_n206), .c(new_n197), .out0(new_n249));
  nand22aa1n09x5               g154(.a(new_n226), .b(new_n246), .o1(new_n250));
  oaoi03aa1n02x5               g155(.a(\a[24] ), .b(\b[23] ), .c(new_n235), .o1(new_n251));
  aoi013aa1n09x5               g156(.a(new_n251), .b(new_n230), .c(new_n232), .d(new_n238), .o1(new_n252));
  aoai13aa1n12x5               g157(.a(new_n252), .b(new_n250), .c(new_n249), .d(new_n214), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  xnrc02aa1n12x5               g159(.a(\b[24] ), .b(\a[25] ), .out0(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  xnbna2aa1n03x5               g161(.a(new_n256), .b(new_n248), .c(new_n254), .out0(\s[25] ));
  nor042aa1n03x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n256), .b(new_n253), .c(new_n185), .d(new_n247), .o1(new_n260));
  xnrc02aa1n02x5               g165(.a(\b[25] ), .b(\a[26] ), .out0(new_n261));
  aoi012aa1n02x7               g166(.a(new_n261), .b(new_n260), .c(new_n259), .o1(new_n262));
  tech160nm_fiaoi012aa1n02p5x5 g167(.a(new_n255), .b(new_n248), .c(new_n254), .o1(new_n263));
  nano22aa1n02x4               g168(.a(new_n263), .b(new_n259), .c(new_n261), .out0(new_n264));
  nor002aa1n02x5               g169(.a(new_n262), .b(new_n264), .o1(\s[26] ));
  nor042aa1n06x5               g170(.a(\b[26] ), .b(\a[27] ), .o1(new_n266));
  nanp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  nanb02aa1n02x5               g172(.a(new_n266), .b(new_n267), .out0(new_n268));
  nor042aa1n06x5               g173(.a(new_n261), .b(new_n255), .o1(new_n269));
  nano32aa1n03x7               g174(.a(new_n250), .b(new_n269), .c(new_n194), .d(new_n211), .out0(new_n270));
  aoai13aa1n12x5               g175(.a(new_n270), .b(new_n193), .c(new_n124), .d(new_n181), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .c(new_n259), .carry(new_n272));
  aobi12aa1n18x5               g177(.a(new_n272), .b(new_n253), .c(new_n269), .out0(new_n273));
  xobna2aa1n03x5               g178(.a(new_n268), .b(new_n271), .c(new_n273), .out0(\s[27] ));
  inv000aa1d42x5               g179(.a(new_n266), .o1(new_n275));
  nano23aa1n02x4               g180(.a(new_n239), .b(new_n221), .c(new_n217), .d(new_n232), .out0(new_n276));
  nanp02aa1n02x5               g181(.a(new_n276), .b(new_n215), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n269), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n272), .b(new_n278), .c(new_n277), .d(new_n252), .o1(new_n279));
  aoai13aa1n03x5               g184(.a(new_n267), .b(new_n279), .c(new_n185), .d(new_n270), .o1(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[27] ), .b(\a[28] ), .out0(new_n281));
  tech160nm_fiaoi012aa1n02p5x5 g186(.a(new_n281), .b(new_n280), .c(new_n275), .o1(new_n282));
  aoi022aa1n06x5               g187(.a(new_n271), .b(new_n273), .c(\b[26] ), .d(\a[27] ), .o1(new_n283));
  nano22aa1n03x7               g188(.a(new_n283), .b(new_n275), .c(new_n281), .out0(new_n284));
  nor002aa1n02x5               g189(.a(new_n282), .b(new_n284), .o1(\s[28] ));
  norp02aa1n02x5               g190(.a(new_n281), .b(new_n268), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n286), .b(new_n279), .c(new_n185), .d(new_n270), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[28] ), .b(\a[29] ), .out0(new_n289));
  tech160nm_fiaoi012aa1n02p5x5 g194(.a(new_n289), .b(new_n287), .c(new_n288), .o1(new_n290));
  aobi12aa1n06x5               g195(.a(new_n286), .b(new_n271), .c(new_n273), .out0(new_n291));
  nano22aa1n03x7               g196(.a(new_n291), .b(new_n288), .c(new_n289), .out0(new_n292));
  nor002aa1n02x5               g197(.a(new_n290), .b(new_n292), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norp03aa1n02x5               g199(.a(new_n289), .b(new_n281), .c(new_n268), .o1(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n279), .c(new_n185), .d(new_n270), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[29] ), .b(\b[28] ), .c(new_n288), .carry(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[29] ), .b(\a[30] ), .out0(new_n298));
  tech160nm_fiaoi012aa1n02p5x5 g203(.a(new_n298), .b(new_n296), .c(new_n297), .o1(new_n299));
  aobi12aa1n06x5               g204(.a(new_n295), .b(new_n271), .c(new_n273), .out0(new_n300));
  nano22aa1n03x7               g205(.a(new_n300), .b(new_n297), .c(new_n298), .out0(new_n301));
  norp02aa1n03x5               g206(.a(new_n299), .b(new_n301), .o1(\s[30] ));
  norb03aa1n03x5               g207(.a(new_n286), .b(new_n298), .c(new_n289), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n279), .c(new_n185), .d(new_n270), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[30] ), .b(\b[29] ), .c(new_n297), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  aoi012aa1n02x7               g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n06x5               g212(.a(new_n303), .b(new_n271), .c(new_n273), .out0(new_n308));
  nano22aa1n03x7               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  nor002aa1n02x5               g214(.a(new_n307), .b(new_n309), .o1(\s[31] ));
  xnrb03aa1n02x5               g215(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g216(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g218(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aob012aa1n02x5               g219(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(new_n315));
  oai012aa1n02x5               g220(.a(new_n315), .b(\b[4] ), .c(\a[5] ), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g222(.a(\a[6] ), .o1(new_n318));
  oaoi03aa1n02x5               g223(.a(new_n318), .b(new_n119), .c(new_n316), .o1(new_n319));
  xnbna2aa1n03x5               g224(.a(new_n319), .b(new_n117), .c(new_n114), .out0(\s[7] ));
  oaoi03aa1n02x5               g225(.a(\a[7] ), .b(\b[6] ), .c(new_n319), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


