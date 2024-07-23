// Benchmark "adder" written by ABC on Thu Jul 18 12:10:01 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n315, new_n317, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  norp02aa1n04x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nand22aa1n12x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nor042aa1n04x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nand02aa1d28x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nano23aa1n09x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .d(new_n99), .out0(new_n103));
  inv040aa1d32x5               g008(.a(\a[5] ), .o1(new_n104));
  inv040aa1d28x5               g009(.a(\b[4] ), .o1(new_n105));
  nanp02aa1n24x5               g010(.a(new_n105), .b(new_n104), .o1(new_n106));
  oaoi03aa1n02x5               g011(.a(\a[6] ), .b(\b[5] ), .c(new_n106), .o1(new_n107));
  oai022aa1n02x5               g012(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n108));
  aoi022aa1n06x5               g013(.a(new_n103), .b(new_n107), .c(new_n99), .d(new_n108), .o1(new_n109));
  orn002aa1n03x5               g014(.a(\a[2] ), .b(\b[1] ), .o(new_n110));
  nand02aa1n08x5               g015(.a(\b[0] ), .b(\a[1] ), .o1(new_n111));
  nand02aa1d16x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  aob012aa1n06x5               g017(.a(new_n110), .b(new_n111), .c(new_n112), .out0(new_n113));
  nor002aa1n04x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  nand42aa1n04x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  norb02aa1n06x5               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nor002aa1n04x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nanp02aa1n04x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  norb02aa1n06x4               g023(.a(new_n118), .b(new_n117), .out0(new_n119));
  nanp03aa1d12x5               g024(.a(new_n113), .b(new_n116), .c(new_n119), .o1(new_n120));
  oai012aa1n04x7               g025(.a(new_n115), .b(new_n117), .c(new_n114), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\a[6] ), .o1(new_n123));
  nanb02aa1n02x5               g028(.a(\b[5] ), .b(new_n123), .out0(new_n124));
  nand42aa1n02x5               g029(.a(new_n124), .b(new_n122), .o1(new_n125));
  nand02aa1d06x5               g030(.a(\b[4] ), .b(\a[5] ), .o1(new_n126));
  nanp02aa1n02x5               g031(.a(new_n106), .b(new_n126), .o1(new_n127));
  nona22aa1n03x5               g032(.a(new_n103), .b(new_n125), .c(new_n127), .out0(new_n128));
  aoai13aa1n12x5               g033(.a(new_n109), .b(new_n128), .c(new_n120), .d(new_n121), .o1(new_n129));
  xorc02aa1n02x5               g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  aoi012aa1n02x5               g035(.a(new_n98), .b(new_n129), .c(new_n130), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  inv000aa1d42x5               g037(.a(\a[9] ), .o1(new_n133));
  inv040aa1d32x5               g038(.a(\b[9] ), .o1(new_n134));
  xroi22aa1d04x5               g039(.a(new_n133), .b(\b[8] ), .c(new_n134), .d(\a[10] ), .out0(new_n135));
  oaoi03aa1n02x5               g040(.a(new_n97), .b(new_n134), .c(new_n98), .o1(new_n136));
  aob012aa1n03x5               g041(.a(new_n136), .b(new_n129), .c(new_n135), .out0(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n09x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nanp02aa1n12x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  tech160nm_fiaoi012aa1n05x5   g045(.a(new_n139), .b(new_n137), .c(new_n140), .o1(new_n141));
  xnrb03aa1n03x5               g046(.a(new_n141), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  oao003aa1n02x5               g047(.a(new_n97), .b(new_n134), .c(new_n98), .carry(new_n143));
  nor022aa1n16x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanp02aa1n09x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nano23aa1n09x5               g050(.a(new_n139), .b(new_n144), .c(new_n145), .d(new_n140), .out0(new_n146));
  tech160nm_fioai012aa1n03p5x5 g051(.a(new_n145), .b(new_n144), .c(new_n139), .o1(new_n147));
  aobi12aa1n06x5               g052(.a(new_n147), .b(new_n146), .c(new_n143), .out0(new_n148));
  nona23aa1n02x4               g053(.a(new_n145), .b(new_n140), .c(new_n139), .d(new_n144), .out0(new_n149));
  norb02aa1n02x5               g054(.a(new_n135), .b(new_n149), .out0(new_n150));
  nanp02aa1n03x5               g055(.a(new_n129), .b(new_n150), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n151), .b(new_n148), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  tech160nm_finand02aa1n05x5   g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  tech160nm_fiaoi012aa1n05x5   g060(.a(new_n154), .b(new_n152), .c(new_n155), .o1(new_n156));
  xnrb03aa1n03x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n03x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n159), .b(new_n158), .out0(new_n160));
  tech160nm_fioai012aa1n03p5x5 g065(.a(new_n147), .b(new_n149), .c(new_n136), .o1(new_n161));
  nor002aa1d24x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nand42aa1n06x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nano23aa1n09x5               g068(.a(new_n154), .b(new_n162), .c(new_n163), .d(new_n155), .out0(new_n164));
  aoai13aa1n06x5               g069(.a(new_n164), .b(new_n161), .c(new_n129), .d(new_n150), .o1(new_n165));
  oa0012aa1n12x5               g070(.a(new_n163), .b(new_n162), .c(new_n154), .o(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n160), .b(new_n165), .c(new_n167), .out0(\s[15] ));
  nand42aa1n02x5               g073(.a(new_n165), .b(new_n167), .o1(new_n169));
  nor022aa1n04x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanp02aa1n03x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  aoai13aa1n03x5               g077(.a(new_n172), .b(new_n158), .c(new_n169), .d(new_n160), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(new_n169), .b(new_n160), .o1(new_n174));
  nona22aa1n02x4               g079(.a(new_n174), .b(new_n172), .c(new_n158), .out0(new_n175));
  nanp02aa1n02x5               g080(.a(new_n175), .b(new_n173), .o1(\s[16] ));
  nano23aa1n06x5               g081(.a(new_n158), .b(new_n170), .c(new_n171), .d(new_n159), .out0(new_n177));
  oa0012aa1n02x5               g082(.a(new_n171), .b(new_n170), .c(new_n158), .o(new_n178));
  tech160nm_fiaoi012aa1n05x5   g083(.a(new_n178), .b(new_n177), .c(new_n166), .o1(new_n179));
  nand02aa1d06x5               g084(.a(new_n177), .b(new_n164), .o1(new_n180));
  inv000aa1n02x5               g085(.a(new_n180), .o1(new_n181));
  aobi12aa1n06x5               g086(.a(new_n179), .b(new_n181), .c(new_n161), .out0(new_n182));
  nano22aa1n12x5               g087(.a(new_n180), .b(new_n135), .c(new_n146), .out0(new_n183));
  nand02aa1d06x5               g088(.a(new_n129), .b(new_n183), .o1(new_n184));
  xorc02aa1n02x5               g089(.a(\a[17] ), .b(\b[16] ), .out0(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n184), .c(new_n182), .out0(\s[17] ));
  inv000aa1d42x5               g091(.a(\a[17] ), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(\b[16] ), .b(new_n187), .out0(new_n188));
  tech160nm_fioai012aa1n05x5   g093(.a(new_n179), .b(new_n148), .c(new_n180), .o1(new_n189));
  aoai13aa1n02x5               g094(.a(new_n185), .b(new_n189), .c(new_n129), .d(new_n183), .o1(new_n190));
  xnrc02aa1n02x5               g095(.a(\b[17] ), .b(\a[18] ), .out0(new_n191));
  xobna2aa1n03x5               g096(.a(new_n191), .b(new_n190), .c(new_n188), .out0(\s[18] ));
  inv000aa1d42x5               g097(.a(\a[18] ), .o1(new_n193));
  xroi22aa1d04x5               g098(.a(new_n187), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n189), .c(new_n129), .d(new_n183), .o1(new_n195));
  oai022aa1n04x7               g100(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n196));
  oaib12aa1n09x5               g101(.a(new_n196), .b(new_n193), .c(\b[17] ), .out0(new_n197));
  nor042aa1n06x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nand02aa1n04x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  xnbna2aa1n03x5               g106(.a(new_n201), .b(new_n195), .c(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  tech160nm_finand02aa1n05x5   g108(.a(new_n195), .b(new_n197), .o1(new_n204));
  nor042aa1n06x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nand02aa1n04x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(new_n205), .b(new_n206), .out0(new_n207));
  aoai13aa1n04x5               g112(.a(new_n207), .b(new_n198), .c(new_n204), .d(new_n201), .o1(new_n208));
  tech160nm_finand02aa1n03p5x5 g113(.a(new_n204), .b(new_n201), .o1(new_n209));
  nona22aa1n02x4               g114(.a(new_n209), .b(new_n207), .c(new_n198), .out0(new_n210));
  nanp02aa1n03x5               g115(.a(new_n210), .b(new_n208), .o1(\s[20] ));
  nona23aa1d18x5               g116(.a(new_n206), .b(new_n199), .c(new_n198), .d(new_n205), .out0(new_n212));
  oai012aa1n06x5               g117(.a(new_n206), .b(new_n205), .c(new_n198), .o1(new_n213));
  oai012aa1d24x5               g118(.a(new_n213), .b(new_n212), .c(new_n197), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n214), .o1(new_n215));
  nano23aa1n09x5               g120(.a(new_n198), .b(new_n205), .c(new_n206), .d(new_n199), .out0(new_n216));
  nanb03aa1n02x5               g121(.a(new_n191), .b(new_n216), .c(new_n185), .out0(new_n217));
  aoai13aa1n06x5               g122(.a(new_n215), .b(new_n217), .c(new_n184), .d(new_n182), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  xnrc02aa1n12x5               g125(.a(\b[20] ), .b(\a[21] ), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  tech160nm_fixnrc02aa1n05x5   g127(.a(\b[21] ), .b(\a[22] ), .out0(new_n223));
  aoai13aa1n02x7               g128(.a(new_n223), .b(new_n220), .c(new_n218), .d(new_n222), .o1(new_n224));
  nand42aa1n02x5               g129(.a(new_n218), .b(new_n222), .o1(new_n225));
  nona22aa1n03x5               g130(.a(new_n225), .b(new_n223), .c(new_n220), .out0(new_n226));
  nanp02aa1n03x5               g131(.a(new_n226), .b(new_n224), .o1(\s[22] ));
  nor042aa1n03x5               g132(.a(new_n223), .b(new_n221), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\a[22] ), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\b[21] ), .o1(new_n230));
  oao003aa1n02x5               g135(.a(new_n229), .b(new_n230), .c(new_n220), .carry(new_n231));
  aoi012aa1n02x5               g136(.a(new_n231), .b(new_n214), .c(new_n228), .o1(new_n232));
  nand03aa1n02x5               g137(.a(new_n194), .b(new_n228), .c(new_n216), .o1(new_n233));
  aoai13aa1n06x5               g138(.a(new_n232), .b(new_n233), .c(new_n184), .d(new_n182), .o1(new_n234));
  xorb03aa1n02x5               g139(.a(new_n234), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g140(.a(\b[22] ), .b(\a[23] ), .o1(new_n236));
  xorc02aa1n12x5               g141(.a(\a[23] ), .b(\b[22] ), .out0(new_n237));
  xnrc02aa1n02x5               g142(.a(\b[23] ), .b(\a[24] ), .out0(new_n238));
  aoai13aa1n02x7               g143(.a(new_n238), .b(new_n236), .c(new_n234), .d(new_n237), .o1(new_n239));
  nand42aa1n02x5               g144(.a(new_n234), .b(new_n237), .o1(new_n240));
  nona22aa1n03x5               g145(.a(new_n240), .b(new_n238), .c(new_n236), .out0(new_n241));
  nanp02aa1n03x5               g146(.a(new_n241), .b(new_n239), .o1(\s[24] ));
  norb02aa1n02x7               g147(.a(new_n237), .b(new_n238), .out0(new_n243));
  inv030aa1n02x5               g148(.a(new_n243), .o1(new_n244));
  nano32aa1n02x4               g149(.a(new_n244), .b(new_n194), .c(new_n228), .d(new_n216), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n189), .c(new_n129), .d(new_n183), .o1(new_n246));
  oaoi03aa1n02x5               g151(.a(\a[18] ), .b(\b[17] ), .c(new_n188), .o1(new_n247));
  inv020aa1n02x5               g152(.a(new_n213), .o1(new_n248));
  aoai13aa1n06x5               g153(.a(new_n228), .b(new_n248), .c(new_n216), .d(new_n247), .o1(new_n249));
  inv000aa1n02x5               g154(.a(new_n231), .o1(new_n250));
  orn002aa1n03x5               g155(.a(\a[23] ), .b(\b[22] ), .o(new_n251));
  oaoi03aa1n02x5               g156(.a(\a[24] ), .b(\b[23] ), .c(new_n251), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  aoai13aa1n02x7               g158(.a(new_n253), .b(new_n244), .c(new_n249), .d(new_n250), .o1(new_n254));
  nanb02aa1n03x5               g159(.a(new_n254), .b(new_n246), .out0(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  tech160nm_fixorc02aa1n05x5   g162(.a(\a[25] ), .b(\b[24] ), .out0(new_n258));
  xorc02aa1n12x5               g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  aoai13aa1n03x5               g165(.a(new_n260), .b(new_n257), .c(new_n255), .d(new_n258), .o1(new_n261));
  nand22aa1n03x5               g166(.a(new_n255), .b(new_n258), .o1(new_n262));
  nona22aa1n02x4               g167(.a(new_n262), .b(new_n260), .c(new_n257), .out0(new_n263));
  nanp02aa1n02x5               g168(.a(new_n263), .b(new_n261), .o1(\s[26] ));
  and002aa1n18x5               g169(.a(new_n259), .b(new_n258), .o(new_n265));
  nanp02aa1n02x5               g170(.a(new_n254), .b(new_n265), .o1(new_n266));
  oai022aa1n02x5               g171(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n267));
  aob012aa1n02x5               g172(.a(new_n267), .b(\b[25] ), .c(\a[26] ), .out0(new_n268));
  nano22aa1n03x7               g173(.a(new_n233), .b(new_n265), .c(new_n243), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n189), .c(new_n129), .d(new_n183), .o1(new_n270));
  nand23aa1n04x5               g175(.a(new_n266), .b(new_n270), .c(new_n268), .o1(new_n271));
  xorb03aa1n02x5               g176(.a(new_n271), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .out0(new_n275));
  aoai13aa1n03x5               g180(.a(new_n275), .b(new_n273), .c(new_n271), .d(new_n274), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n243), .b(new_n231), .c(new_n214), .d(new_n228), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n265), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n268), .b(new_n278), .c(new_n277), .d(new_n253), .o1(new_n279));
  aobi12aa1n06x5               g184(.a(new_n269), .b(new_n184), .c(new_n182), .out0(new_n280));
  oaih12aa1n02x5               g185(.a(new_n274), .b(new_n279), .c(new_n280), .o1(new_n281));
  nona22aa1n03x5               g186(.a(new_n281), .b(new_n275), .c(new_n273), .out0(new_n282));
  nanp02aa1n03x5               g187(.a(new_n276), .b(new_n282), .o1(\s[28] ));
  norb02aa1n02x5               g188(.a(new_n274), .b(new_n275), .out0(new_n284));
  oaih12aa1n02x5               g189(.a(new_n284), .b(new_n279), .c(new_n280), .o1(new_n285));
  inv000aa1n03x5               g190(.a(new_n273), .o1(new_n286));
  oaoi03aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n286), .o1(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  nona22aa1n03x5               g193(.a(new_n285), .b(new_n287), .c(new_n288), .out0(new_n289));
  aoai13aa1n03x5               g194(.a(new_n288), .b(new_n287), .c(new_n271), .d(new_n284), .o1(new_n290));
  nanp02aa1n03x5               g195(.a(new_n290), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g196(.a(new_n111), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g197(.a(new_n274), .b(new_n288), .c(new_n275), .out0(new_n293));
  oao003aa1n02x5               g198(.a(\a[28] ), .b(\b[27] ), .c(new_n286), .carry(new_n294));
  oaoi03aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .o1(new_n295));
  tech160nm_fixorc02aa1n03p5x5 g200(.a(\a[30] ), .b(\b[29] ), .out0(new_n296));
  inv000aa1d42x5               g201(.a(new_n296), .o1(new_n297));
  aoai13aa1n02x7               g202(.a(new_n297), .b(new_n295), .c(new_n271), .d(new_n293), .o1(new_n298));
  oaih12aa1n02x5               g203(.a(new_n293), .b(new_n279), .c(new_n280), .o1(new_n299));
  nona22aa1n03x5               g204(.a(new_n299), .b(new_n295), .c(new_n297), .out0(new_n300));
  nanp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[30] ));
  nanp02aa1n02x5               g206(.a(new_n295), .b(new_n296), .o1(new_n302));
  oai012aa1n02x5               g207(.a(new_n302), .b(\b[29] ), .c(\a[30] ), .o1(new_n303));
  nano23aa1n02x4               g208(.a(new_n288), .b(new_n275), .c(new_n296), .d(new_n274), .out0(new_n304));
  oaih12aa1n02x5               g209(.a(new_n304), .b(new_n279), .c(new_n280), .o1(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  nona22aa1n03x5               g211(.a(new_n305), .b(new_n306), .c(new_n303), .out0(new_n307));
  aoai13aa1n02x7               g212(.a(new_n306), .b(new_n303), .c(new_n271), .d(new_n304), .o1(new_n308));
  nanp02aa1n03x5               g213(.a(new_n308), .b(new_n307), .o1(\s[31] ));
  xorb03aa1n02x5               g214(.a(new_n113), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi012aa1n02x5               g215(.a(new_n117), .b(new_n113), .c(new_n118), .o1(new_n311));
  xnrc02aa1n02x5               g216(.a(new_n311), .b(new_n116), .out0(\s[4] ));
  xobna2aa1n03x5               g217(.a(new_n127), .b(new_n120), .c(new_n121), .out0(\s[5] ));
  nanp02aa1n02x5               g218(.a(new_n120), .b(new_n121), .o1(new_n314));
  oaoi03aa1n02x5               g219(.a(new_n104), .b(new_n105), .c(new_n314), .o1(new_n315));
  xnbna2aa1n03x5               g220(.a(new_n315), .b(new_n122), .c(new_n124), .out0(\s[6] ));
  oaoi03aa1n02x5               g221(.a(\a[6] ), .b(\b[5] ), .c(new_n315), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g223(.a(new_n101), .b(new_n317), .c(new_n102), .o1(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g225(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


